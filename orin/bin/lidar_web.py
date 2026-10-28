#!/usr/bin/env python3
"""Web control panel for the MID360 rig: scan, record, view - from a phone.

Answers "how do I run the rig without an SSH session": one page with buttons.
It does not reimplement anything - every button shells out to the same script
the aliases call, so the CLI stays the single source of truth:

    Scan    -> lidar_scan.py  -> map_web.py   (static dense cloud)
    Record  -> ros2 bag record
    Stop    -> lio_run.sh     -> map_web.py   (FAST-LIO2 map of the bag)
    Live    -> lio_live.py    (FAST-LIO2 on the live topic, map + robot
               position refreshed every second; Stop -> map_walls + map_web)

Finished pages are written to ~/maps and served from this same port, so the
result opens in the browser that pressed the button.

    lidar-web           serve on port 8000
    lidar-web 8080      different port
"""
import glob
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time

from flask import Flask, Response, jsonify, request, send_from_directory

HOME = os.path.expanduser("~")
BIN = os.path.join(HOME, "bin")
MAPS = os.path.join(HOME, "maps")
SCANS = os.path.join(HOME, "scans")
BAGS = os.path.join(HOME, "bags")

# A Flask worker inherits no interactive shell, so ROS has to be sourced per
# subprocess - the same three lines ~/.bashrc runs.
ROS = ("source /opt/ros/humble/setup.bash; "
       f"[ -f {HOME}/livox_ws/install/setup.bash ] && "
       f"source {HOME}/livox_ws/install/setup.bash; "
       "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}; ")

app = Flask(__name__)

# ponytail: one job at a time, held in module state. There is one LiDAR on the
# rig, so a second concurrent job has nothing to read - a queue would be
# machinery for a case that cannot happen.
start_lock = threading.Lock()
job = {"state": "idle", "step": "", "log": [], "result": None,
       "t0": 0.0, "total": 0.0, "name": ""}
rec = {"proc": None, "bag": None, "t0": 0.0}
# The step sh() is running, so Cancel can kill it; cancel stops later steps.
cur = {"proc": None, "cancel": False}


def log(line):
    job["log"].append(line.rstrip("\n"))
    del job["log"][:-200]


def sh(cmd, step):
    """Run cmd in bash, streaming its output into the job log. Returns rc."""
    if cur["cancel"]:
        return 1
    job["step"] = step
    log(f"$ {step}")
    # Own process group, so Cancel takes down lio_run.sh's children too.
    p = subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1,
                         start_new_session=True)
    cur["proc"] = p
    for line in p.stdout:
        log(line)
    rc = p.wait()
    cur["proc"] = None
    return rc


def fail(e):
    if cur["cancel"]:
        log("cancelled - any recorded bag is kept in ~/bags")
        job.update(state="idle", step="cancelled")
    else:
        job.update(state="error", step=str(e))


def begin(state, name, total=0.0):
    job.update(state=state, name=name, step="starting", log=[], result=None,
               t0=time.time(), total=total)
    cur["cancel"] = False


def finish(html):
    job.update(state="done", step="ready",
               result="/maps/" + os.path.basename(html))


def spawn(fn, *a):
    threading.Thread(target=fn, args=a, daemon=True).start()


def safe(name, default):
    name = re.sub(r"\s+", "-", (name or "").strip())      # "two floors" -> two-floors
    name = re.sub(r"[^A-Za-z0-9_-]", "", name)
    return name or default


# ------------------------------------------------------------------ scan ---

def do_scan(seconds, voxel):
    try:
        ply = os.path.join(SCANS, time.strftime("scan_%Y%m%d_%H%M%S.ply"))
        if sh(ROS + f"exec python3 {BIN}/lidar_scan.py {seconds:g} "
                    f"--voxel {voxel:g} --out {ply} --no-publish",
              f"accumulating {seconds:g}s at {voxel*100:g} cm"):
            raise RuntimeError("scan failed - is the driver running?")
        job.update(total=0.0)
        out = os.path.join(MAPS, os.path.basename(ply)[:-4] + ".html")
        if sh(f"exec /usr/bin/python3 {BIN}/map_web.py {ply} -o {out}",
              "levelling and building the viewer"):
            raise RuntimeError("map_web failed")
        finish(out)
    except Exception as e:
        fail(e)


@app.post("/api/scan")
def api_scan():
    body = request.get_json(silent=True) or {}
    seconds = min(max(float(body.get("seconds", 30)), 2.0), 300.0)
    voxel = min(max(float(body.get("voxel", 0.02)), 0.005), 0.2)
    with start_lock:
        if busy():
            return jsonify(error="a job is already running"), 409
        begin("scanning", "scan", seconds)
    spawn(do_scan, seconds, voxel)
    return jsonify(ok=True)


# ---------------------------------------------------------------- record ---

def busy():
    return job["state"] in ("scanning", "recording", "processing")


@app.post("/api/record/start")
def api_record_start():
    body = request.get_json(silent=True) or {}
    name = safe(body.get("name"), "walk")
    with start_lock:
        if busy():
            return jsonify(error="a job is already running"), 409
        bag = os.path.join(BAGS, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(BAGS, exist_ok=True)
        begin("recording", name)
        log(f"recording to {bag}")
        log("walk a loop that RETURNS TO THE START - loop closure is the point")
        rec["proc"] = subprocess.Popen(
            ["bash", "-c", ROS + f"exec ros2 bag record -o {bag} "
                                 f"/livox/lidar /livox/imu"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rec["bag"] = bag
        rec["t0"] = time.time()
    return jsonify(ok=True, bag=os.path.basename(bag))


def do_stop():
    bag = rec["bag"]
    base = os.path.basename(bag)
    try:
        # ros2 bag record only closes the file cleanly on SIGINT.
        job.update(state="processing", step="closing the bag", total=0.0)
        rec["proc"].send_signal(signal.SIGINT)
        try:
            rec["proc"].wait(timeout=60)
        except subprocess.TimeoutExpired:
            rec["proc"].kill()
        rec["proc"] = None
        if sh(f"exec du -sh {bag}", "recorded"):
            raise RuntimeError("the bag was not written")
        if sh(f"exec {BIN}/lio_run.sh {bag}",
              "FAST-LIO2: replaying the walk (takes as long as the walk)"):
            raise RuntimeError("lio_run failed - see the log")
        out = os.path.join(MAPS, base + ".html")
        if sh(f"exec /usr/bin/python3 {BIN}/map_web.py {MAPS}/{base} -o {out} "
              f"--title {base}", "building the viewer"):
            raise RuntimeError("map_web failed")
        finish(out)
    except Exception as e:
        fail(e)


@app.post("/api/record/stop")
def api_record_stop():
    # Flip the state inside the lock, not in the worker: a double tap on a
    # laggy phone would otherwise start two do_stop threads on one bag.
    with start_lock:
        if job["state"] != "recording" or not rec["proc"]:
            return jsonify(error="not recording"), 409
        job.update(state="processing", step="closing the bag", total=0.0)
    spawn(do_stop)
    return jsonify(ok=True)


@app.post("/api/cancel")
def api_cancel():
    with start_lock:
        if job["state"] not in ("scanning", "processing"):
            return jsonify(error="nothing to cancel"), 409
        cur["cancel"] = True
        p = cur["proc"]
        if p and p.poll() is None:
            os.killpg(p.pid, signal.SIGTERM)
    return jsonify(ok=True)


# ---------------------------------------------------------------- status ---

@app.get("/api/status")
def api_status():
    s = {k: job[k] for k in ("state", "step", "result", "total", "name")}
    # A finished map deleted since (from the Library or a shell) must not be
    # auto-opened again - that showed a full-screen "Not Found".
    if s["result"] and not os.path.exists(os.path.join(MAPS, s["result"][len("/maps/"):])):
        s["result"] = None
    s["elapsed"] = 0.0 if not job["t0"] else round(time.time() - job["t0"], 1)
    s["log"] = job["log"][-60:]
    return jsonify(s)


@app.get("/api/list")
def api_list():
    out = []
    for f in sorted(glob.glob(os.path.join(MAPS, "*.html")),
                    key=os.path.getmtime, reverse=True):
        out.append({"name": os.path.basename(f),
                    "url": "/maps/" + os.path.basename(f),
                    "mb": round(os.path.getsize(f) / 1e6, 1),
                    "when": time.strftime("%Y-%m-%d %H:%M",
                                          time.localtime(os.path.getmtime(f)))})
    return jsonify(out)


# -------------------------------------------------------------- navigate ---
# Nav2 runs on the robot PC (Foxy). The panel sends it floor maps
# (robot_send.sh) and poses/goals (nav_link.py -> DDS -> web_bridge.py there),
# and runs the Jetson's half of navigation: /scan + the lidar mount TF.
navs = {"link": None, "lidar": None, "sent": None,
        "state": {"status": "not started", "path": [], "robot": None}}
nav_lock = threading.Lock()


def nav_up():
    """Start the Jetson's nav processes if they are not running."""
    with nav_lock:
        if navs["lidar"] is None and navs["link"] is None:
            # First use since the panel started: clear children a previous
            # panel left behind (they run in their own sessions, so a panel
            # restart does not take them down).
            for name in ("lidar_nav.launch.py", "nav_link.py", "pointcloud_to_laserscan_node"):
                subprocess.run(["pkill", "-f", name])
        if navs["lidar"] is None or navs["lidar"].poll() is not None:
            navs["lidar"] = subprocess.Popen(
                ["bash", "-c", ROS + f"exec ros2 launch {BIN}/lidar_nav.launch.py"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
        if navs["link"] is None or navs["link"].poll() is not None:
            # System python: nav_link.py's ROS messages break under kiss_env.
            p = subprocess.Popen(
                ["bash", "-c", ROS + f"exec /usr/bin/python3 {BIN}/nav_link.py"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, text=True, bufsize=1,
                start_new_session=True)
            navs["link"] = p
            spawn(nav_read, p)


def nav_read(p):
    for line in p.stdout:
        try:
            navs["state"] = json.loads(line)
        except ValueError:
            pass


def nav_cmd(c):
    nav_up()
    navs["link"].stdin.write(json.dumps(c) + "\n")
    navs["link"].stdin.flush()


def floor_maps():
    out = []
    for y in sorted(glob.glob(os.path.join(MAPS, "*", "slam_output", "latest",
                                           "walls", "floor*.yaml")),
                    key=os.path.getmtime, reverse=True):
        d = {}
        for line in open(y):
            k, _, v = line.partition(":")
            d[k.strip()] = v.strip()
        rel = os.path.relpath(y, MAPS)
        out.append({"id": rel,
                    "name": rel.split(os.sep)[0] + " " + os.path.basename(y)[:-5],
                    "png": "/maps/" + rel[:-5] + ".png",
                    "res": float(d["resolution"]),
                    "origin": [float(v) for v in d["origin"].strip("[]").split(",")[:2]]})
    return out


@app.get("/api/nav/maps")
def api_nav_maps():
    return jsonify(maps=floor_maps(), sent=navs["sent"])


@app.post("/api/nav/send")
def api_nav_send():
    mid = (request.get_json(silent=True) or {}).get("map")
    if mid not in {m["id"] for m in floor_maps()}:
        return jsonify(error="unknown map"), 404
    try:
        r = subprocess.run(["bash", "-c", f"exec {BIN}/robot_send.sh "
                                          f"{shlex.quote(os.path.join(MAPS, mid))}"],
                           capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        return jsonify(error="send timed out"), 504
    lines = (r.stdout + r.stderr).strip().splitlines()
    sent = [l[6:] for l in lines if l.startswith("sent: ")]
    if r.returncode or not sent:
        return jsonify(error=lines[-1] if lines else "send failed"), 502
    navs["sent"] = mid
    nav_cmd({"cmd": "load", "path": sent[0]})
    return jsonify(ok=True, remote=sent[0])


@app.post("/api/nav/cmd")
def api_nav_cmd():
    b = request.get_json(silent=True) or {}
    if b.get("cmd") not in ("pose", "plan", "go", "stop"):
        return jsonify(error="unknown command"), 400
    try:
        c = {"cmd": b["cmd"]}
        if b["cmd"] != "stop":
            c.update(x=float(b["x"]), y=float(b["y"]), yaw=float(b.get("yaw", 0)))
    except (KeyError, TypeError, ValueError):
        return jsonify(error="x, y needed"), 400
    nav_cmd(c)
    return jsonify(ok=True)


@app.get("/api/nav/state")
def api_nav_state():
    nav_up()
    return jsonify(dict(navs["state"], sent=navs["sent"]))


# ------------------------------------------------------------------ live ---
# lio_live.py runs FAST-LIO2 on the live topic and rewrites ~/maps/live/
# state.json + changed map tiles every 0.2 s; the Live tab polls those. It runs beside
# any job (it only reads the topic), so it has its own state, not job's.
# A run is a map ({"map": true}: the Live tab, mapmaker.py) or odometry only
# (lio_live --nav: what lio_relay starts for navigation, with no map to build or
# save). Asking for a map while an odometry run goes replaces that run.
LIVE = os.path.join(MAPS, "live")
live = {"proc": None, "log": [], "map": False}


def live_read(p):
    for line in p.stdout:
        live["log"].append(line.rstrip("\n"))
        del live["log"][:-100]


def live_running():
    return live["proc"] is not None and live["proc"].poll() is None


@app.post("/api/live/start")
def api_live_start():
    want_map = bool((request.get_json(silent=True) or {}).get("map"))
    with start_lock:
        if live_running() and want_map and not live["map"]:
            p = live["proc"]
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        if live_running():
            return jsonify(error="live SLAM is already running"), 409
        # A panel restart does not take its children down (own session).
        for leftover in (f"{BIN}/lio_live.py", "fast_lio/fastlio_mapping"):
            subprocess.run(["pkill", "-INT", "-f", leftover])
        live["log"] = []
        live["map"] = want_map
        live["proc"] = subprocess.Popen(
            # System python: FAST-LIO's Odometry message breaks under
            # kiss_env's numpy 2 (as nav_link.py's do).
            ["bash", "-c", ROS + f"exec /usr/bin/python3 -u "
                                 f"{BIN}/lio_live.py --out {LIVE}"
                                 + ("" if want_map else " --nav")],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True)
        spawn(live_read, live["proc"])
    return jsonify(ok=True)


def named(d, name):
    """live_<stamp> -> <name>_<stamp>: the name typed in the Live tab, so a
    run can be told apart later; the stamp keeps it unique and sortable."""
    base = os.path.basename(d)
    if name == "live" or not base.startswith("live_"):
        return d
    new = os.path.join(os.path.dirname(d), f"{name}_{base[5:]}")
    if os.path.exists(new):
        return d
    os.rename(d, new)
    return new


def do_live_stop(p, post, one_floor=False, name="live"):
    """Let lio_live save, then clean ghosts and build floor maps + viewer."""
    try:
        try:
            p.wait(timeout=180)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            raise RuntimeError("live SLAM did not save in time")
        saved = [l[6:] for l in live["log"] if l.startswith("SAVED ")]
        if saved:
            saved[-1] = named(saved[-1], name)
            live["log"].append(f"saved as {os.path.basename(saved[-1])}")
        if not post:
            return
        for l in live["log"][-9:]:
            log(l)
        if not saved:
            raise RuntimeError("nothing saved - too short a session?")
        d = saved[-1]
        base = os.path.basename(d)
        # Remove ghosts (people who walked past) before anything is built
        # from the map. Non-fatal: runs saved without a scan log skip it.
        if sh(f"exec /usr/bin/python3 {BIN}/lio_clean.py {shlex.quote(d)} --no-rebuild",
              "removing people who walked past (ghosts)"):
            log("  ghost removal skipped - the map is saved uncleaned")
        one = ""
        if one_floor:
            # One level walked: the path's height is pure drift - remove it,
            # and make one wall plan (a ceiling would count as a 2nd storey).
            if sh(f"exec {shlex.quote(sys.executable)} {BIN}/kiss_flatten.py "
                  f"{shlex.quote(d)}", "one floor: removing height drift"):
                raise RuntimeError("kiss_flatten failed")
            one = " --one-floor"
        sh(f"exec /usr/bin/python3 {BIN}/map_walls.py {shlex.quote(d)}{one}",
           "floor maps for Navigate")
        out = os.path.join(MAPS, base + ".html")
        if sh(f"exec /usr/bin/python3 {BIN}/map_web.py {shlex.quote(d)} -o {out} "
              f"--title {base}{one}", "building the viewer"):
            raise RuntimeError("map_web failed")
        finish(out)
    except Exception as e:
        if post:
            fail(e)
        else:
            live["log"].append(f"error: {e}")


@app.post("/api/live/stop")
def api_live_stop():
    with start_lock:
        if not live_running():
            return jsonify(error="live SLAM is not running"), 409
        p = live["proc"]
        body = request.get_json(silent=True) or {}
        one_floor = bool(body.get("one_floor"))
        name = safe(body.get("name"), "live")
        os.killpg(p.pid, signal.SIGINT)
        # Post-processing goes through the job pipeline (Progress card, opens
        # the viewer) unless a recording/scan owns it - then just save.
        post = live["map"] and not busy()
        if post:
            begin("processing", name, 0.0)
            job["step"] = "closing the map (last loop-closure check)"
    spawn(do_live_stop, p, post, one_floor, name)
    return jsonify(ok=True)


@app.get("/api/live/state")
def api_live_state():
    try:
        with open(os.path.join(LIVE, "state.json")) as f:
            s = json.load(f)
    except (OSError, ValueError):
        s = {"status": "not started"}
    s["running"] = live_running()
    s["age"] = round(time.time() - s.get("updated", 0), 1) if "updated" in s else None
    s["log"] = live["log"][-6:]
    return jsonify(s)


@app.get("/live")
def live_page():
    return PAGE.replace("__TAB__", "live")


# --------------------------------------------------------------- thermal ---
# The thermal camera (thermal_node.py) writes its latest colour frame to a
# JPEG in RAM 9 times a second; this streams it to the page as MJPEG while
# the Thermal toggle is on. No ROS in here: the panel only reads a file.
THERMAL_JPEG = "/dev/shm/thermal_color.jpg"


def thermal_age():
    try:
        return time.time() - os.path.getmtime(THERMAL_JPEG)
    except OSError:
        return None


@app.get("/api/thermal/status")
def api_thermal_status():
    age = thermal_age()
    return jsonify(ok=age is not None and age < 3.0,
                   age=None if age is None else round(age, 1))


@app.get("/thermal.mjpg")
def thermal_stream():
    def frames():
        last = None
        idle = 0.0
        while idle < 30.0:                 # camera gone 30 s: end; the page retries
            try:
                m = os.path.getmtime(THERMAL_JPEG)
            except OSError:
                m = None
            if m is not None and m != last:
                try:
                    with open(THERMAL_JPEG, "rb") as f:
                        jpg = f.read()
                except OSError:
                    jpg = b""
                if jpg:
                    last, idle = m, 0.0
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    continue
            time.sleep(0.04)
            idle += 0.04
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store"})


@app.get("/maps/<path:p>")
def maps(p):
    return send_from_directory(MAPS, p)


@app.get("/")
def index():
    return PAGE.replace("__TAB__", "scan")


@app.get("/record")
def record_page():
    return PAGE.replace("__TAB__", "record")


@app.get("/nav")
def nav_page():
    return PAGE.replace("__TAB__", "nav")


# The page: panel.html next to this script; __TAB__ picks the open tab.
PAGE = open(os.path.join(BIN, "panel.html"), encoding="utf-8").read()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    for d in (MAPS, SCANS, BAGS):
        os.makedirs(d, exist_ok=True)
    ts = subprocess.run(["tailscale", "ip", "-4"], capture_output=True,
                        text=True).stdout.split("\n")[0].strip()
    print(f"  control panel on port {port}")
    if ts:
        print(f"    tailnet : http://{ts}:{port}/")
    print(f"    local   : http://127.0.0.1:{port}/")
    print("  Ctrl-C to stop\n")
    # Bind all interfaces so the tailnet address works; the tailnet is private.
    # threaded=True matters: the status poll must answer while a job runs.
    app.run(host="0.0.0.0", port=port, threaded=True)
