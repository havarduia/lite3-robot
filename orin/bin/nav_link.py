#!/usr/bin/env python3
"""The web panel's ROS side for navigation. Runs under the SYSTEM python.

lidar_web.py runs in ~/kiss_env (numpy 2), where ROS message types with
fixed arrays (PoseWithCovarianceStamped) break, so the panel keeps ROS out of
its own process: it starts this, writes one JSON command per line to stdin,
and reads one JSON state line from stdout whenever something changes.

    commands: {"cmd": "pose"|"plan"|"go", "x":, "y":, "yaw":}
              {"cmd": "stop"}   {"cmd": "load", "path": "<map yaml on robot PC>"}
    state:    {"status": str, "path": [[x, y], ...], "robot": [x, y, yaw] | null}

The robot PC's web_bridge.py turns these topics into Nav2 calls.
"""
import json
import math
import sys
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

state = {"status": "waiting for the robot PC", "path": [], "robot": None}


def emit():
    print(json.dumps(state), flush=True)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fill(pose, x, y, yaw):
    pose.position.x, pose.position.y = float(x), float(y)
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)


def main():
    rclpy.init()
    node = rclpy.create_node("web_nav_link")
    pub = {
        "plan": node.create_publisher(PoseStamped, "/web/plan_goal", 1),
        "go": node.create_publisher(PoseStamped, "/web/go", 1),
        "stop": node.create_publisher(Empty, "/web/stop", 1),
        "load": node.create_publisher(String, "/web/load_map", 1),
        "pose": node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1),
    }

    def on_path(m):
        state["path"] = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                         for p in m.poses]
        emit()

    def on_status(m):
        state["status"] = m.data
        emit()

    def on_robot(m):
        p = m.pose.pose
        state["robot"] = [round(p.position.x, 3), round(p.position.y, 3),
                          round(yaw_of(p.orientation), 3)]
        emit()

    node.create_subscription(Path, "/web/plan", on_path, 1)
    node.create_subscription(String, "/web/status", on_status, 10)
    # AMCL publishes its pose transient-local; match it to get the last one.
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_robot,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def commands():
        for line in sys.stdin:
            try:
                c = json.loads(line)
                cmd = c["cmd"]
                if cmd in ("plan", "go"):
                    m = PoseStamped()
                    m.header.frame_id = "map"
                    m.header.stamp = node.get_clock().now().to_msg()
                    fill(m.pose, c["x"], c["y"], c.get("yaw", 0.0))
                    if cmd == "plan":
                        state["path"] = []
                elif cmd == "pose":
                    m = PoseWithCovarianceStamped()
                    m.header.frame_id = "map"
                    m.header.stamp = node.get_clock().now().to_msg()
                    fill(m.pose.pose, c["x"], c["y"], c.get("yaw", 0.0))
                    cov = [0.0] * 36
                    cov[0] = cov[7] = 0.25          # 0.5 m std in x, y
                    cov[35] = 0.07                  # ~15 deg std in yaw
                    m.pose.covariance = cov
                elif cmd == "stop":
                    m = Empty()
                elif cmd == "load":
                    m = String(data=c["path"])
                else:
                    raise KeyError(cmd)
                pub[cmd].publish(m)
                state["status"] = f"sent {cmd}"
            except Exception as e:                  # bad command: say, keep going
                state["status"] = f"bad command: {e}"
            emit()
        rclpy.shutdown()                            # panel closed our stdin

    threading.Thread(target=commands, daemon=True).start()
    emit()
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == "__main__":
    main()
