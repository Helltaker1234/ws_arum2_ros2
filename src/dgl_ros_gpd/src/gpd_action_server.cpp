#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gpd/candidate/hand.h>
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using SampleGraspPoses = dgl_ros_interfaces::action::SampleGraspPoses;
using GoalHandle = rclcpp_action::ServerGoalHandle<SampleGraspPoses>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using GpdPointCloud = gpd::util::PointCloudRGB;

double finite_score_or_lowest(const std::unique_ptr<gpd::candidate::Hand> & hand)
{
  const double score = hand->getScore();
  return std::isfinite(score) ? score : std::numeric_limits<double>::lowest();
}
}  // namespace

class GpdActionServer : public rclcpp::Node
{
public:
  GpdActionServer()
  : Node("dgl_ros_gpd"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/camera/depth/points_cleaned");
    action_name_ = declare_parameter<std::string>("action_name", "/sample_grasp_poses");
    gpd_config_path_ = declare_parameter<std::string>("gpd_config_path", "");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "");
    camera_view_point_ = declare_parameter<std::vector<double>>(
      "camera_view_point", {0.0, 0.0, 0.0});
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.2);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 2.0);
    minimum_points_ = declare_parameter<int>("minimum_points", 100);
    max_candidates_ = declare_parameter<int>("max_candidates", 20);
    cost_scale_ = declare_parameter<double>("cost_scale", 1.0);
    cost_offset_ = declare_parameter<double>("cost_offset", 0.0);
    publish_observation_ = declare_parameter<bool>("publish_observation", false);
    observation_topic_ = declare_parameter<std::string>("observation_topic", "observation");

    validate_parameters();
    detector_ = std::make_unique<gpd::GraspDetector>(gpd_config_path_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GpdActionServer::cloud_callback, this, std::placeholders::_1));

    if (publish_observation_) {
      observation_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        observation_topic_, rclcpp::SensorDataQoS());
    }

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<SampleGraspPoses>(
      this, action_name_, std::bind(&GpdActionServer::handle_goal, this, _1, _2),
      std::bind(&GpdActionServer::handle_cancel, this, _1),
      std::bind(&GpdActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(
      get_logger(), "GPD server ready: cloud '%s', action '%s'", input_topic_.c_str(),
      action_name_.c_str());
  }

  ~GpdActionServer() override
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void validate_parameters() const
  {
    if (input_topic_.empty() || action_name_.empty()) {
      throw std::invalid_argument("input_topic and action_name must not be empty");
    }
    if (gpd_config_path_.empty() || !std::filesystem::is_regular_file(gpd_config_path_)) {
      throw std::invalid_argument(
              "gpd_config_path must point to an existing GPD configuration file");
    }
    if (camera_view_point_.size() != 3) {
      throw std::invalid_argument("camera_view_point must contain exactly three values");
    }
    if (tf_timeout_sec_ < 0.0 || max_cloud_age_sec_ < 0.0 || minimum_points_ < 1 ||
      max_candidates_ < 0 || !std::isfinite(cost_scale_) || !std::isfinite(cost_offset_))
    {
      throw std::invalid_argument("invalid GPD server parameter");
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud without frame_id");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    }

    if (observation_pub_) {
      observation_pub_->publish(*msg);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const SampleGraspPoses::Goal> goal)
  {
    if (inference_active_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejecting grasp request: GPD inference is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      get_logger(), "Accepted grasp request%s%s", goal->action_name.empty() ? "" : ": ",
      goal->action_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> &)
  {
    RCLCPP_INFO(get_logger(), "Received grasp request cancellation");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread(&GpdActionServer::execute, this, goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    struct ActiveGuard
    {
      explicit ActiveGuard(std::atomic_bool & active) : active_(active) {}
      ~ActiveGuard() {active_.store(false);}
      std::atomic_bool & active_;
    } guard(inference_active_);

    auto result = std::make_shared<SampleGraspPoses::Result>();
    try {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!latest_cloud_) {
          result->grasp_state = "no_point_cloud";
          goal_handle->abort(result);
          return;
        }
        cloud_msg = *latest_cloud_;
      }

      if (cloud_is_stale(cloud_msg)) {
        result->grasp_state = "stale_point_cloud";
        goal_handle->abort(result);
        return;
      }
      if (goal_handle->is_canceling()) {
        result->grasp_state = "canceled";
        goal_handle->canceled(result);
        return;
      }

      auto cloud = to_gpd_cloud(cloud_msg);
      if (cloud->size() < static_cast<std::size_t>(minimum_points_)) {
        RCLCPP_WARN(get_logger(), "Cloud has only %zu valid points", cloud->size());
        result->grasp_state = "insufficient_points";
        goal_handle->abort(result);
        return;
      }

      Eigen::Matrix3Xd view_points(3, 1);
      view_points.col(0) = camera_origin_in_cloud_frame(cloud_msg);
      gpd::util::Cloud gpd_cloud(cloud, 0, view_points);

      RCLCPP_INFO(get_logger(), "Running GPD on %zu points", cloud->size());
      detector_->preprocessPointCloud(gpd_cloud);
      auto grasps = detector_->detectGrasps(gpd_cloud);
      std::sort(
        grasps.begin(), grasps.end(),
        [](const auto & lhs, const auto & rhs) {
          return finite_score_or_lowest(lhs) > finite_score_or_lowest(rhs);
        });
      if (max_candidates_ > 0 && grasps.size() > static_cast<std::size_t>(max_candidates_)) {
        grasps.resize(static_cast<std::size_t>(max_candidates_));
      }

      if (goal_handle->is_canceling()) {
        result->grasp_state = "canceled";
        goal_handle->canceled(result);
        return;
      }

      auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
      feedback->grasp_candidates.reserve(grasps.size());
      feedback->costs.reserve(grasps.size());
      for (const auto & grasp : grasps) {
        const double score = grasp->getScore();
        if (!std::isfinite(score)) {
          continue;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = cloud_msg.header;
        const Eigen::Vector3d & position = grasp->getPosition();
        const Eigen::Quaterniond orientation(grasp->getOrientation());
        pose.pose.position.x = position.x();
        pose.pose.position.y = position.y();
        pose.pose.position.z = position.z();
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        feedback->grasp_candidates.push_back(std::move(pose));
        feedback->costs.push_back(cost_offset_ - cost_scale_ * score);
      }

      goal_handle->publish_feedback(feedback);
      result->grasp_state = feedback->grasp_candidates.empty() ? "no_grasps" : "succeeded";
      goal_handle->succeed(result);
      RCLCPP_INFO(
        get_logger(), "GPD returned %zu grasp candidates", feedback->grasp_candidates.size());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "GPD inference failed: %s", error.what());
      result->grasp_state = std::string("error: ") + error.what();
      if (rclcpp::ok() && goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }
  }

  bool cloud_is_stale(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    if (max_cloud_age_sec_ == 0.0 || rclcpp::Time(cloud.header.stamp).nanoseconds() == 0) {
      return false;
    }
    const double age = (now() - rclcpp::Time(cloud.header.stamp)).seconds();
    if (age > max_cloud_age_sec_) {
      RCLCPP_WARN(get_logger(), "Latest point cloud is %.3f seconds old", age);
      return true;
    }
    return false;
  }

  GpdPointCloud::Ptr to_gpd_cloud(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    PointCloud xyz;
    pcl::fromROSMsg(msg, xyz);
    PointCloud valid;
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(xyz, valid, valid_indices);

    auto rgba = GpdPointCloud::Ptr(new GpdPointCloud);
    pcl::copyPointCloud(valid, *rgba);
    rgba->width = static_cast<std::uint32_t>(rgba->size());
    rgba->height = 1;
    rgba->is_dense = true;
    return rgba;
  }

  Eigen::Vector3d camera_origin_in_cloud_frame(
    const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    if (camera_frame_.empty() || camera_frame_ == cloud.header.frame_id) {
      return Eigen::Vector3d(
        camera_view_point_[0], camera_view_point_[1], camera_view_point_[2]);
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        cloud.header.frame_id, camera_frame_, cloud.header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      return Eigen::Vector3d(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z);
    } catch (const tf2::TransformException & error) {
      throw std::runtime_error(
              "cannot transform camera frame '" + camera_frame_ + "' to cloud frame '" +
              cloud.header.frame_id + "': " + error.what());
    }
  }

  std::string input_topic_;
  std::string action_name_;
  std::string gpd_config_path_;
  std::string camera_frame_;
  std::string observation_topic_;
  std::vector<double> camera_view_point_;
  double tf_timeout_sec_;
  double max_cloud_age_sec_;
  int minimum_points_;
  int max_candidates_;
  double cost_scale_;
  double cost_offset_;
  bool publish_observation_;

  std::mutex cloud_mutex_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_cloud_;
  std::atomic_bool inference_active_{false};
  std::thread worker_thread_;
  std::unique_ptr<gpd::GraspDetector> detector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observation_pub_;
  rclcpp_action::Server<SampleGraspPoses>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpdActionServer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dgl_ros_gpd"), "Failed to start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
