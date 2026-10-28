// terrain_level - FAST-LIO2's cloud and pose, levelled, for CMU terrainAnalysis; and its
// terrain map as heights, for a Nav2 costmap (nav_stack_migration.md, steps 2-3).
//
// FAST-LIO2's world frame (camera_init) is the IMU frame at start-up, tilted ~20 deg nose down
// on this mount; terrainAnalysis takes z as up. Both are turned by the rotation that takes
// FAST-LIO2's own "up" (minus /lio/gravity) to z, fixed once from the first gravity_n
// estimates: terrainAnalysis keeps a map in this frame, so it must not move under it.
//
//   /cloud_registered, /Odometry  ->  /registered_scan, /state_estimation   (frame 'level')
//   /terrain_map                  ->  /terrain_h  (z := height above the local ground, frame 'odom')
//
// C++ because the Python version cost half a core on the 520 kB clouds (big_walk 2026-10-28).
#include <cstring>
#include <memory>
#include <numeric>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using geometry_msgs::msg::Vector3Stamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

namespace {

int offset_of(const PointCloud2 & m, const std::string & name)
{
  for (const auto & f : m.fields) {
    if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {return f.offset;}
  }
  return -1;
}

}  // namespace

class TerrainLevel : public rclcpp::Node
{
public:
  TerrainLevel()
  : Node("terrain_level")
  {
    gravity_n_ = declare_parameter("gravity_n", 20);
    cloud_pub_ = create_publisher<PointCloud2>("/registered_scan", 5);
    odom_pub_ = create_publisher<Odometry>("/state_estimation", 5);
    height_pub_ = create_publisher<PointCloud2>("/terrain_h", 2);
    gravity_sub_ = create_subscription<Vector3Stamped>(
      "/lio/gravity", 20, [this](Vector3Stamped::ConstSharedPtr m) {gravity(*m);});
    cloud_sub_ = create_subscription<PointCloud2>(
      "/cloud_registered", 5, [this](PointCloud2::UniquePtr m) {cloud(std::move(m));});
    odom_sub_ = create_subscription<Odometry>(
      "/Odometry", 20, [this](Odometry::UniquePtr m) {odom(std::move(m));});
    terrain_sub_ = create_subscription<PointCloud2>(
      "/terrain_map", 2, [this](PointCloud2::UniquePtr m) {terrain(std::move(m));});
  }

private:
  void gravity(const Vector3Stamped & m)
  {
    if (levelled_) {return;}
    ups_.emplace_back(-m.vector.x, -m.vector.y, -m.vector.z);
    if (static_cast<int>(ups_.size()) < gravity_n_) {return;}
    Eigen::Vector3d up = std::accumulate(ups_.begin(), ups_.end(), Eigen::Vector3d::Zero().eval());
    q_ = Eigen::Quaterniond::FromTwoVectors(up.normalized(), Eigen::Vector3d::UnitZ());
    r_ = q_.toRotationMatrix().cast<float>();
    levelled_ = true;
    RCLCPP_INFO(get_logger(), "levelled by %.1f deg", acos(up.normalized().z()) * 180.0 / M_PI);
  }

  void cloud(PointCloud2::UniquePtr m)
  {
    if (!levelled_) {return;}
    if (offset_of(*m, "x") != 0 || offset_of(*m, "y") != 4 || offset_of(*m, "z") != 8) {
      RCLCPP_ERROR_ONCE(get_logger(), "/cloud_registered is not x, y, z at 0, 4, 8: not levelled");
      return;
    }
    for (size_t i = 0; i + 12 <= m->data.size(); i += m->point_step) {
      Eigen::Vector3f p;
      std::memcpy(p.data(), &m->data[i], 12);
      p = r_ * p;
      std::memcpy(&m->data[i], p.data(), 12);
    }
    m->header.frame_id = "level";
    cloud_pub_->publish(std::move(m));
  }

  void odom(Odometry::UniquePtr m)
  {
    if (!levelled_) {return;}
    auto & p = m->pose.pose.position;
    auto & o = m->pose.pose.orientation;
    Eigen::Vector3d v = q_ * Eigen::Vector3d(p.x, p.y, p.z);
    Eigen::Quaterniond q = q_ * Eigen::Quaterniond(o.w, o.x, o.y, o.z);
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    o.x = q.x(); o.y = q.y(); o.z = q.z(); o.w = q.w();
    m->header.frame_id = "level";
    odom_pub_->publish(std::move(m));
  }

  // level and odom share x, y (odom is level without its height); the costmap marks by z
  void terrain(PointCloud2::UniquePtr m)
  {
    const int z = offset_of(*m, "z"), h = offset_of(*m, "intensity");
    if (z < 0 || h < 0) {
      RCLCPP_ERROR_ONCE(get_logger(), "/terrain_map has no float z or intensity");
      return;
    }
    for (size_t i = 0; i + m->point_step <= m->data.size(); i += m->point_step) {
      std::memcpy(&m->data[i + z], &m->data[i + h], 4);
    }
    m->header.frame_id = "odom";
    height_pub_->publish(std::move(m));
  }

  int gravity_n_;
  bool levelled_ = false;
  std::vector<Eigen::Vector3d> ups_;
  Eigen::Quaterniond q_;
  Eigen::Matrix3f r_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_, height_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<Vector3Stamped>::SharedPtr gravity_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_, terrain_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainLevel>());
  rclcpp::shutdown();
}
