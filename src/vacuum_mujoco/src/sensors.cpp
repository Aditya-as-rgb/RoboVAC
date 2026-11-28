#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace vacuum_mujoco
{
class Sensors : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData *) override
  {
    node_ = std::move(node);
    model_ = model;
    scan_site_ = mj_name2id(model, mjOBJ_SITE, "base_scan");
    body_ = mj_name2id(model, mjOBJ_BODY, "vacuum");
    accel_ = sensor_address("accelerometer");
    gyro_ = sensor_address("gyroscope");
    attitude_ = sensor_address("attitude");
    if (scan_site_ < 0 || body_ < 0 || accel_ < 0 || gyro_ < 0 || attitude_ < 0) {
      return false;
    }
    const std::string prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
    const auto truth = node_->get_parameter(prefix + "ground_truth");
    ground_truth_ = truth.as_bool();
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      ground_truth_ ? "/odom" : "/odom_truth", 10);
    if (ground_truth_) {
      // Match odometry's queue depth so TF doesn't carry seconds of old poses under load.
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(
        node_, tf2_ros::DynamicBroadcasterQoS(10));
    }
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    for (size_t i = 0; i < bumpers_.size(); ++i) {
      const std::string side = i == 0 ? "left" : "right";
      bumpers_[i] = node_->create_publisher<ros_gz_interfaces::msg::Contacts>(
        "/bumper_" + side + "/contact", 10);
    }
    for (int id = 0; id < model->nbody; ++id) {
      const int mocap = model->body_mocapid[id];
      const char * name = mj_id2name(model, mjOBJ_BODY, id);
      if (mocap >= 0 && name && std::string(name).rfind("dirt_", 0) == 0) {
        dirt_.push_back({name, mocap, model->body_pos[3 * id + 2]});
      }
    }
    if (!dirt_.empty()) {
      dirt_state_ = std::make_shared<DirtState>();
      dirt_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/dirt/state", rclcpp::QoS(1).transient_local(),
        [state = dirt_state_](const std_msgs::msg::String & msg) {
          std::unordered_set<std::string> remaining;
          static const std::regex patch_name("\"(dirt_[0-9]+)\"");
          for (std::sregex_iterator match(msg.data.begin(), msg.data.end(), patch_name), end;
            match != end; ++match)
          {
            remaining.insert((*match)[1]);
          }
          std::lock_guard<std::mutex> lock(state->mutex);
          state->remaining = std::move(remaining);
          state->received = true;
        });
    }
    auto cmd_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/vacuum_drive/cmd_vel", 10);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [cmd_pub, clock = node_->get_clock()](const geometry_msgs::msg::Twist & msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = clock->now();
        stamped.twist = msg;
        cmd_pub->publish(stamped);
      });
    return true;
  }

  void cleanup() override
  {
    cmd_sub_.reset();
    dirt_sub_.reset();
  }

  void pre_step(mjData * data) override
  {
    if (dirt_state_) {
      std::lock_guard<std::mutex> lock(dirt_state_->mutex);
      if (dirt_state_->received) {
        for (const auto & dirt : dirt_) {
          data->mocap_pos[3 * dirt.mocap + 2] =
            dirt_state_->remaining.count(dirt.name) ? dirt.z : -2.;
        }
      }
    }
    if (data->time < last_odom_) {
      last_odom_ = last_scan_ = last_imu_ = -1.;
    }
    const auto stamp = rclcpp::Time(static_cast<int64_t>(data->time * 1e9), RCL_ROS_TIME);
    if (data->time >= last_odom_ + .0199) {
      last_odom_ = data->time;
      publish_odom(data, stamp);
      publish_contacts(data, stamp);
    }
    if (data->time >= last_imu_ + .0099) {
      last_imu_ = data->time;
      publish_imu(data, stamp);
    }
    if (data->time >= last_scan_ + .1999) {
      last_scan_ = data->time;
      publish_scan(data, stamp);
    }
  }

private:
  struct Dirt
  {
    std::string name;
    int mocap;
    double z;
  };

  struct DirtState
  {
    std::mutex mutex;
    std::unordered_set<std::string> remaining;
    bool received = false;
  };

  int sensor_address(const char * name) const
  {
    const int id = mj_name2id(model_, mjOBJ_SENSOR, name);
    return id < 0 ? -1 : model_->sensor_adr[id];
  }

  template<typename Vector>
  static void vector(Vector & result, const mjtNum * value)
  {
    result.x = value[0];
    result.y = value[1];
    result.z = value[2];
  }

  static void quaternion(geometry_msgs::msg::Quaternion & result, const mjtNum * value)
  {
    result.w = value[0];
    result.x = value[1];
    result.y = value[2];
    result.z = value[3];
  }

  void publish_odom(const mjData * data, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";
    vector(msg.pose.pose.position, data->xpos + 3 * body_);
    quaternion(msg.pose.pose.orientation, data->xquat + 4 * body_);
    mjtNum velocity[6];
    mj_objectVelocity(model_, data, mjOBJ_XBODY, body_, velocity, 1);
    vector(msg.twist.twist.angular, velocity);
    vector(msg.twist.twist.linear, velocity + 3);
    odom_pub_->publish(msg);
    if (ground_truth_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = msg.header;
      transform.child_frame_id = msg.child_frame_id;
      vector(transform.transform.translation, data->xpos + 3 * body_);
      transform.transform.rotation = msg.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
  }

  void publish_imu(const mjData * data, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "imu_link";
    quaternion(msg.orientation, data->sensordata + attitude_);
    vector(msg.angular_velocity, data->sensordata + gyro_);
    vector(msg.linear_acceleration, data->sensordata + accel_);
    imu_pub_->publish(msg);
  }

  void publish_scan(const mjData * data, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_scan";
    constexpr int rays = 360;
    msg.angle_min = 0.;
    msg.angle_increment = 2. * std::acos(-1.) / rays;
    msg.angle_max = (rays - 1) * msg.angle_increment;
    msg.range_min = .1;
    msg.range_max = 10.;
    msg.scan_time = .2;
    msg.ranges.reserve(rays);
    const auto * origin = data->site_xpos + 3 * scan_site_;
    const auto * rotation = data->site_xmat + 9 * scan_site_;
    // World collision group only. Exclude robot, rug and visual-only dirt.
    const mjtByte groups[mjNGROUP] = {1, 0, 0, 0, 0, 0};
    for (int i = 0; i < rays; ++i) {
      const double angle = i * msg.angle_increment;
      const mjtNum local[3] = {std::cos(angle), std::sin(angle), 0.};
      mjtNum ray[3];
      mju_mulMatVec(ray, rotation, local, 3, 3);
      int geom = -1;
      const auto distance = mj_ray(model_, data, origin, ray, groups, 1, body_, &geom, nullptr);
      msg.ranges.push_back(
        distance >= msg.range_min && distance <= msg.range_max ?
        static_cast<float>(distance) : std::numeric_limits<float>::infinity());
    }
    scan_pub_->publish(msg);
  }

  void publish_contacts(const mjData * data, const rclcpp::Time & stamp)
  {
    std::array<ros_gz_interfaces::msg::Contacts, 2> messages;
    for (int i = 0; i < data->ncon; ++i) {
      const auto & contact = data->contact[i];
      for (int which = 0; which < 2; ++which) {
        const int geom = contact.geom[which];
        const int other = contact.geom[1 - which];
        if (geom < 0 || other < 0) {
          continue;
        }
        const char * raw = mj_id2name(model_, mjOBJ_GEOM, geom);
        const std::string name = raw ? raw : "";
        if (name.rfind("bumper_", 0) != 0) {
          continue;
        }
        const int address = sensor_address(name.c_str());
        if (address < 0 || data->sensordata[address] <= 0.) {
          continue;
        }
        auto & hit = messages[name.rfind("bumper_left_", 0) == 0 ? 0 : 1].contacts.emplace_back();
        hit.collision1.id = geom + 1;
        hit.collision1.name = name;
        hit.collision1.type = ros_gz_interfaces::msg::Entity::COLLISION;
        hit.collision2.id = other + 1;
        const char * other_name = mj_id2name(model_, mjOBJ_GEOM, other);
        if (other_name) {
          hit.collision2.name = other_name;
        }
        hit.collision2.type = ros_gz_interfaces::msg::Entity::COLLISION;
        vector(hit.positions.emplace_back(), contact.pos);
        vector(hit.normals.emplace_back(), contact.frame);
        hit.depths.push_back(std::max(0., -contact.dist));
      }
    }
    for (size_t i = 0; i < bumpers_.size(); ++i) {
      messages[i].header.stamp = stamp;
      messages[i].header.frame_id = "base_link";
      bumpers_[i]->publish(messages[i]);
    }
  }

  rclcpp::Node::SharedPtr node_;
  const mjModel * model_ = nullptr;
  int body_, scan_site_, accel_, gyro_, attitude_;
  double last_odom_ = -1., last_scan_ = -1., last_imu_ = -1.;
  bool ground_truth_ = true;
  std::vector<Dirt> dirt_;
  std::shared_ptr<DirtState> dirt_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::array<rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr, 2> bumpers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dirt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};
}  // namespace vacuum_mujoco

PLUGINLIB_EXPORT_CLASS(vacuum_mujoco::Sensors,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
