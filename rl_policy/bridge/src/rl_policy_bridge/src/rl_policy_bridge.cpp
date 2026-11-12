#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <yaml-cpp/yaml.h>
#include <onnxruntime_cxx_api.h>

#include <airsim_ros/VelCmd.h>

namespace {

constexpr int kImgWidth = 80;
constexpr int kImgHeight = 20;
constexpr float kPitchUpperDeg = 40.0f;
constexpr float kPitchSpanDeg = 59.0f;
constexpr float kDepthFar = 50.0f;
constexpr float kDepthMin = 0.3f;
constexpr float kDepthMax = 30.0f;
constexpr double kPi = 3.14159265358979323846;

struct PoseSample {
  ros::Time stamp;
  Eigen::Vector3d position;
};

struct PathResult {
  bool valid = false;
  Eigen::Vector3d closest;
  Eigen::Vector3d tangent;
  int segment = -1;
  double distance = 0.0;
};

double wrap_angle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

class RlPolicyBridge {
 public:
  RlPolicyBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    const char* policy_root_env = std::getenv("RL_POLICY_ROOT");
    const std::string policy_root = policy_root_env ? policy_root_env : "/workspace/rl_bridge";
    const std::string default_path = policy_root_env
        ? policy_root + "/bridge/path_point/1-3_center.yaml"
        : policy_root + "/path_point/1-3_center.yaml";
    const std::string default_model = policy_root_env
        ? policy_root + "/policy/model_11100.onnx"
        : policy_root + "/models/policy.onnx";
    pnh_.param<std::string>("path_file", path_file_, default_path);
    pnh_.param<std::string>("model_path", model_path_, default_model);
    pnh_.param<std::string>("vel_topic", vel_topic_, "/airsim_node/drone_1/vel_body_cmd");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/airsim_node/drone_1/debug/pose_gt");
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/airsim_node/drone_1/lidar");
    pnh_.param("target_speed", target_speed_, 8.0);
    pnh_.param("takeoff_height", takeoff_height_, 1.5);
    pnh_.param("control_rate", control_rate_, 20.0);
    pnh_.param("yaw_gain", yaw_gain_, 1.5);
    pnh_.param("yaw_rate_max_deg", yaw_rate_max_deg_, 60.0);
    pnh_.param("max_speed", max_speed_, 15.0);
    pnh_.param("va", va_, 8);
    pnh_.param("climb_speed", climb_speed_, 0.4);
    pnh_.param("align_gain", align_gain_, 1.0);
    pnh_.param("align_tolerance", align_tolerance_, 0.15);
    pnh_.param("align_settle_sec", align_settle_sec_, 3.0);
    pnh_.param("use_path_z", use_path_z_, false);
    pnh_.param("hold_altitude", hold_altitude_, 2.0);
    pnh_.param("use_fixed_altitude", use_fixed_altitude_, true);

    load_path(path_file_);

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(2);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rl_policy_bridge");
    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path_.c_str(), session_options);
    hx_.assign(192, 0.0f);
    depth_.assign(kImgWidth * kImgHeight, 1.0f);
    state_.assign(14, 0.0f);

    cmd_pub_ = nh_.advertise<airsim_ros::VelCmd>(vel_topic_, 1);
    pose_sub_ = nh_.subscribe(pose_topic_, 1, &RlPolicyBridge::pose_cb, this);
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &RlPolicyBridge::lidar_cb, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &RlPolicyBridge::control_cb, this);

    ROS_INFO("[rl_bridge] path=%s (%zu pts) model=%s target_speed=%.1f",
             path_file_.c_str(), waypoints_.size(), model_path_.c_str(), target_speed_);
  }

 private:
  void load_path(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    const YAML::Node& wps = root["waypoints"];
    if (!wps || !wps.IsSequence()) {
      throw std::runtime_error("path file has no waypoints sequence: " + path);
    }
    for (const auto& wp : wps) {
      if (!wp.IsSequence() || wp.size() < 3) continue;
      waypoints_.emplace_back(wp[0].as<double>(), wp[1].as<double>(), wp[2].as<double>());
    }
    if (waypoints_.size() < 2) throw std::runtime_error("path file has fewer than 2 waypoints");
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = msg->pose;
    const ros::Time arrival = ros::Time::now();
    pose_stamp_ = arrival;
    pose_arrival_ = arrival;
    has_pose_ = true;

    Eigen::Vector3d p(pose_.position.x, pose_.position.y, pose_.position.z);
    pose_history_.push_back({arrival, p});
    while (!pose_history_.empty() &&
           (arrival - pose_history_.front().stamp).toSec() > 1.0) {
      pose_history_.pop_front();
    }
    if (start_z_ == 0.0 && !have_start_z_) {
      start_z_ = p.z();
      have_start_z_ = true;
      ROS_INFO("[rl_bridge] start position: %.3f %.3f %.3f", p.x(), p.y(), p.z());
    }
  }

  void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto& f : msg->fields) {
      if (f.datatype != sensor_msgs::PointField::FLOAT32) continue;
      if (f.name == "x") off_x = static_cast<int>(f.offset);
      if (f.name == "y") off_y = static_cast<int>(f.offset);
      if (f.name == "z") off_z = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) {
      ROS_WARN_THROTTLE(5.0, "[rl_bridge] lidar has no float32 xyz fields");
      return;
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(msg->width) * msg->height);
    const size_t count = static_cast<size_t>(msg->width) * msg->height;
    for (size_t i = 0; i < count; ++i) {
      const uint8_t* base = msg->data.data() + (i / msg->width) * msg->row_step +
                            (i % msg->width) * msg->point_step;
      float x, y, z;
      std::memcpy(&x, base + off_x, sizeof(float));
      std::memcpy(&y, base + off_y, sizeof(float));
      std::memcpy(&z, base + off_z, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      points.emplace_back(x, y, z);
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_ = std::move(points);
    cloud_stamp_ = ros::Time::now();
    cloud_arrival_ = cloud_stamp_;
    has_cloud_ = true;
    ROS_INFO_THROTTLE(1.0, "[rl_bridge] lidar %zu pts stamp=%.3f now=%.3f",
                      cloud_.size(), cloud_stamp_.toSec(), ros::Time::now().toSec());
  }

  PathResult nearest_path(const Eigen::Vector3d& p) const {
    PathResult result;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      const Eigen::Vector3d& a = waypoints_[i];
      const Eigen::Vector3d& b = waypoints_[i + 1];
      const Eigen::Vector3d ab = b - a;
      const double ab2 = ab.squaredNorm();
      if (ab2 < 1e-9) continue;
      const double t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
      const Eigen::Vector3d proj = a + t * ab;
      const double d2 = (proj - p).squaredNorm();
      if (d2 < best) {
        best = d2;
        result.valid = true;
        result.closest = proj;
        result.tangent = ab.normalized();
        result.segment = static_cast<int>(i);
        result.distance = std::sqrt(d2);
      }
    }
    return result;
  }

  void render_depth() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    std::fill(depth_.begin(), depth_.end(), kDepthFar);
    for (const auto& pt : cloud_) {
      const float x = pt.x(), y = pt.y(), z = pt.z();
      const float range = std::sqrt(x * x + y * y + z * z);
      if (range < kDepthMin) continue;
      const float yaw_deg = std::atan2(y, x) * 180.0f / static_cast<float>(kPi);
      const float pitch_deg = std::asin(std::clamp(-z / range, -1.0f, 1.0f)) * 180.0f /
                              static_cast<float>(kPi);
      const int v = static_cast<int>(std::lround(((yaw_deg / 180.0f + 1.0f) * kImgWidth / 2.0f) - 0.5f));
      const int u = static_cast<int>(std::lround(((kPitchUpperDeg - pitch_deg) / kPitchSpanDeg) *
                                                  kImgHeight - 0.5f));
      if (u < 0 || u >= kImgHeight || v < 0 || v >= kImgWidth) continue;
      float& cell = depth_[u * kImgWidth + v];
      if (range < cell) cell = range;
    }
    for (float& d : depth_) {
      d = std::clamp(d, kDepthMin, kDepthMax) / kDepthMax;
    }
  }

  Eigen::Vector3d estimate_world_velocity(const ros::Time& now) const {
    if (pose_history_.size() < 2) return Eigen::Vector3d::Zero();
    const PoseSample& latest = pose_history_.back();
    for (auto it = pose_history_.rbegin(); it != pose_history_.rend(); ++it) {
      const double dt = (latest.stamp - it->stamp).toSec();
      if (dt >= 0.04) {
        if (dt > 0.25 || dt <= 0.0) return Eigen::Vector3d::Zero();
        return (latest.position - it->position) / dt;
      }
    }
    (void)now;
    return Eigen::Vector3d::Zero();
  }

  void run_policy(const Eigen::Vector3d& p, const Eigen::Vector3d& v_world,
                  const Eigen::Vector3d& rpy_yaw_axis, double yaw,
                  const PathResult& path, Eigen::Vector3d* action) {
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d v_bl = R_yaw.transpose() * v_world;
    const Eigen::Vector3d target_v_world = path.tangent * target_speed_;
    const Eigen::Vector3d target_v_bl = R_yaw.transpose() * target_v_world;
    const Eigen::Vector3d delta_p_world = path.closest - p;
    const Eigen::Vector3d delta_p_bl = R_yaw.transpose() * delta_p_world;
    const double dp_norm = delta_p_bl.norm();
    Eigen::Vector3d dp_dir = Eigen::Vector3d::Zero();
    if (dp_norm > 1e-6) dp_dir = delta_p_bl / dp_norm;

    state_[0] = v_bl.x();
    state_[1] = v_bl.y();
    state_[2] = v_bl.z();
    state_[3] = target_v_bl.x();
    state_[4] = target_v_bl.y();
    state_[5] = target_v_bl.z();
    state_[6] = rpy_yaw_axis.x();
    state_[7] = rpy_yaw_axis.y();
    state_[8] = rpy_yaw_axis.z();
    state_[9] = dp_norm;
    state_[10] = dp_dir.x();
    state_[11] = dp_dir.y();
    state_[12] = dp_dir.z();
    state_[13] = 0.5f;

    std::array<int64_t, 4> depth_shape{1, 1, kImgHeight, kImgWidth};
    std::array<int64_t, 2> state_shape{1, 14};
    std::array<int64_t, 2> hx_shape{1, 192};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> inputs;
    inputs.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info, depth_.data(), depth_.size(), depth_shape.data(), depth_shape.size()));
    inputs.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info, state_.data(), state_.size(), state_shape.data(), state_shape.size()));
    inputs.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info, hx_.data(), hx_.size(), hx_shape.data(), hx_shape.size()));

    const char* input_names[] = {"depth", "state", "hx"};
    const char* output_names[] = {"act", "next_hx"};
    auto outputs = ort_session_->Run(Ort::RunOptions{nullptr}, input_names, inputs.data(),
                                     inputs.size(), output_names, 2);
    const float* act = outputs[0].GetTensorMutableData<float>();
    const float* next_hx = outputs[1].GetTensorMutableData<float>();
    *action = Eigen::Vector3d(act[0], act[1], act[2]);
    std::copy(next_hx, next_hx + 192, hx_.begin());
  }

  void publish(double vx, double vy, double vz_up, double yaw_rate_deg, uint8_t stop = 0) {
    airsim_ros::VelCmd cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "drone_1";
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.vz = vz_up;
    cmd.yawRate = yaw_rate_deg;
    cmd.va = va_;
    cmd.stop = stop;
    cmd_pub_.publish(cmd);
  }

  void control_cb(const ros::TimerEvent&) {
    geometry_msgs::Pose pose;
    ros::Time pose_stamp;
    ros::Time cloud_stamp;
    bool has_pose_local = false;
    bool has_cloud_local = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = pose_;
      pose_stamp = pose_stamp_;
      has_pose_local = has_pose_;
      std::lock_guard<std::mutex> clock(cloud_mutex_);
      cloud_stamp = cloud_stamp_;
      has_cloud_local = has_cloud_;
    }

    if (!has_pose_local) {
      publish(0, 0, 0, 0);
      return;
    }

    const ros::Time now = ros::Time::now();
    const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                              pose.orientation.z);
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));

    if (!have_start_z_) {
      start_z_ = p.z();
      have_start_z_ = true;
    }
    const double altitude = start_z_ - p.z();
    if (use_fixed_altitude_ && !path_flattened_) {
      for (auto& wp : waypoints_) wp.z() = start_z_ - hold_altitude_;
      path_flattened_ = true;
      ROS_INFO("[rl_bridge] path flattened at altitude %.2f", hold_altitude_);
    }
    const double target_altitude = use_fixed_altitude_
        ? hold_altitude_
        : std::max(takeoff_height_, start_z_ - waypoints_.front().z());

    Eigen::Vector3d v_world;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v_world = estimate_world_velocity(now);
    }

    if (!altitude_ready_) {
      policy_active_ = false;
      const double err = target_altitude - altitude;
      if (std::fabs(err) > align_tolerance_) {
        align_hold_start_ = ros::Time();
        const double vz = std::clamp(align_gain_ * err + 0.6 * v_world.z(),
                                     -climb_speed_, climb_speed_);
        publish(0.0, 0.0, vz, 0.0);
        ROS_INFO_THROTTLE(1.0, "[rl_bridge] align altitude=%.2f target=%.2f err=%.2f",
                          altitude, target_altitude, err);
        return;
      }
      if (align_hold_start_.isZero()) align_hold_start_ = now;
      if ((now - align_hold_start_).toSec() < align_settle_sec_) {
        publish(0.0, 0.0, 0.0, 0.0);
        ROS_INFO_THROTTLE(1.0, "[rl_bridge] settling altitude=%.2f target=%.2f",
                          altitude, target_altitude);
        return;
      }
      altitude_ready_ = true;
      ROS_INFO("[rl_bridge] altitude aligned at %.2f, enabling policy", altitude);
    }

    if (!has_cloud_local || (now - cloud_stamp).toSec() > 0.5 ||
        (now - pose_stamp).toSec() > 0.5) {
      policy_active_ = false;
      publish(0.0, 0.0, 0.0, 0.0);
      ROS_WARN_THROTTLE(1.0, "[rl_bridge] waiting: have_cloud=%d cloud_age=%.2f pose_age=%.2f",
                        has_cloud_local ? 1 : 0,
                        has_cloud_local ? (now - cloud_stamp).toSec() : -1.0,
                        (now - pose_stamp).toSec());
      return;
    }

    if (!policy_active_) {
      std::fill(hx_.begin(), hx_.end(), 0.0f);
      policy_active_ = true;
      ROS_INFO("[rl_bridge] policy active, altitude=%.2f", altitude);
    }

    render_depth();
    const PathResult path = nearest_path(p);
    if (!path.valid) {
      publish(0, 0, 0, 0);
      return;
    }

    Eigen::Vector3d action;
    run_policy(p, v_world, R.col(2), yaw, path, &action);
    if (!action.allFinite()) {
      publish(0, 0, 0, 0);
      return;
    }
    const double speed = action.norm();
    if (speed > max_speed_) action *= max_speed_ / speed;

    const double yaw_target = std::atan2(path.tangent.y(), path.tangent.x());
    const double yaw_err = wrap_angle(yaw_target - yaw);
    const double yaw_rate_deg = std::clamp(yaw_gain_ * yaw_err * 180.0 / kPi,
                                           -yaw_rate_max_deg_, yaw_rate_max_deg_);

    const double vx = action.x();
    const double vy = action.y();
    const double path_altitude = start_z_ - path.closest.z();
    double vz_up = -action.z();
    if (use_fixed_altitude_) {
      const double alt_err = hold_altitude_ - altitude;
      if (std::fabs(alt_err) < 0.8) {
        vz_up = 0.0;
      } else {
        vz_up = std::clamp(0.4 * alt_err, -0.3, 0.3);
      }
    } else if (use_path_z_) {
      vz_up = std::clamp(0.8 * (path_altitude - altitude), -0.5, 0.5);
    } else {
      if (altitude > path_altitude + 0.35) vz_up = std::min(vz_up, -0.3);
      if (altitude < path_altitude - 0.8) vz_up = std::max(vz_up, 0.3);
      vz_up = std::clamp(vz_up, -1.5, 1.0);
    }
    publish(vx, vy, vz_up, yaw_rate_deg);

    ROS_INFO_THROTTLE(1.0,
                      "[rl_bridge] p=(%.1f,%.1f,%.1f) alt=%.2f vw=(%.1f,%.1f,%.1f) "
                      "act=(%.2f,%.2f,%.2f) dp=%.2f seg=%d yaw_err=%.1fdeg",
                      p.x(), p.y(), p.z(), altitude, v_world.x(), v_world.y(), v_world.z(),
                      vx, vy, vz_up, path.distance, path.segment, yaw_err * 180.0 / kPi);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber lidar_sub_;
  ros::Timer control_timer_;

  std::string path_file_;
  std::string model_path_;
  std::string vel_topic_;
  std::string pose_topic_;
  std::string lidar_topic_;
  double target_speed_ = 8.0;
  double takeoff_height_ = 1.0;
  double control_rate_ = 20.0;
  double yaw_gain_ = 1.5;
  double yaw_rate_max_deg_ = 60.0;
  double max_speed_ = 15.0;
  double climb_speed_ = 0.4;
  double align_gain_ = 1.0;
  double align_tolerance_ = 0.2;
  double align_settle_sec_ = 3.0;
  bool use_path_z_ = false;
  double hold_altitude_ = 2.0;
  bool use_fixed_altitude_ = true;
  bool path_flattened_ = false;
  int va_ = 8;

  std::mutex mutex_;
  geometry_msgs::Pose pose_;
  ros::Time pose_stamp_;
  ros::Time pose_arrival_;
  bool has_pose_ = false;
  std::deque<PoseSample> pose_history_;
  double start_z_ = 0.0;
  bool have_start_z_ = false;
  bool altitude_ready_ = false;
  ros::Time align_hold_start_;

  std::mutex cloud_mutex_;
  std::vector<Eigen::Vector3f> cloud_;
  ros::Time cloud_stamp_;
  ros::Time cloud_arrival_;
  bool has_cloud_ = false;

  std::vector<Eigen::Vector3d> waypoints_;
  bool policy_active_ = false;

  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::vector<float> hx_;
  std::vector<float> depth_;
  std::vector<float> state_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rl_policy_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    RlPolicyBridge bridge(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[rl_bridge] %s", e.what());
    return 1;
  }
  return 0;
}
