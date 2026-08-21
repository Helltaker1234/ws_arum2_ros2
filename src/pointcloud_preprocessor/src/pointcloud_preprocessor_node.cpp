#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
}  // namespace

class PointCloudPreprocessor : public rclcpp::Node
{
public:
  PointCloudPreprocessor()
  : Node("pointcloud_preprocessor"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_  = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/depth/points_cleaned");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.1);

    crop_min_x_ = declare_parameter<double>("crop_min_x", 0.25);
    crop_max_x_ = declare_parameter<double>("crop_max_x", 0.90);
    crop_min_y_ = declare_parameter<double>("crop_min_y", -0.45);
    crop_max_y_ = declare_parameter<double>("crop_max_y", 0.45);
    crop_min_z_ = declare_parameter<double>("crop_min_z", 0.02);
    crop_max_z_ = declare_parameter<double>("crop_max_z", 0.70);

    enable_voxel_grid_ = declare_parameter<bool>("enable_voxel_grid", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.005);

    remove_table_ = declare_parameter<bool>("remove_table", false);
    plane_distance_threshold_ = declare_parameter<double>("plane_distance_threshold", 0.008);
    plane_max_angle_deg_ = declare_parameter<double>("plane_max_angle_deg", 15.0);
    minimum_plane_inliers_ = declare_parameter<int>("minimum_plane_inliers", 500);
    check_table_height_ = declare_parameter<bool>("check_table_height", false);
    expected_table_height_ = declare_parameter<double>("expected_table_height", 0.50);
    table_height_tolerance_ = declare_parameter<double>("table_height_tolerance", 0.03);

    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", false);
    roi_topic_ = declare_parameter<std::string>("roi_topic", "/camera/depth/points_roi");
    table_topic_ =
      declare_parameter<std::string>("table_topic", "/camera/depth/table_plane");

    validate_parameters();

    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    if (publish_debug_clouds_) {
      roi_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(roi_topic_, rclcpp::SensorDataQoS());
      table_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(table_topic_, rclcpp::SensorDataQoS());
    }

    input_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudPreprocessor::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Point-cloud preprocessing: %s -> %s (frame: %s)", input_topic_.c_str(),
      output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void validate_parameters()
  {
    if (target_frame_.empty()) 
    {
      throw std::invalid_argument("target_frame must not be empty");
    }
    if (crop_min_x_ >= crop_max_x_ || crop_min_y_ >= crop_max_y_ || crop_min_z_ >= crop_max_z_)
    {
      throw std::invalid_argument("Each crop minimum must be smaller than its maximum");
    }
    if (enable_voxel_grid_ && voxel_leaf_size_ <= 0.0) 
    {
      throw std::invalid_argument("voxel_leaf_size must be positive");
    }
    if (transform_timeout_sec_ < 0.0 || plane_distance_threshold_ <= 0.0 ||
        plane_max_angle_deg_ <= 0.0 || plane_max_angle_deg_ >= 90.0 ||
        minimum_plane_inliers_ < 3 || table_height_tolerance_ < 0.0)
    {
      throw std::invalid_argument("Invalid transform or table-plane parameter");
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Input cloud has no frame_id");
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_msg;
    try {
      if (msg->header.frame_id == target_frame_) {
        transformed_msg = *msg;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(transform_timeout_sec_));
        tf2::doTransform(*msg, transformed_msg, transform);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Skipping cloud: cannot transform %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    Cloud::Ptr input(new Cloud);
    pcl::fromROSMsg(transformed_msg, *input);

    Cloud::Ptr valid(new Cloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*input, *valid, valid_indices);

    Cloud::Ptr cropped(new Cloud);
    pcl::CropBox<Point> crop;
    crop.setInputCloud(valid);
    crop.setMin(Eigen::Vector4f(
      static_cast<float>(crop_min_x_), static_cast<float>(crop_min_y_),
      static_cast<float>(crop_min_z_), 1.0F));
    crop.setMax(Eigen::Vector4f(
      static_cast<float>(crop_max_x_), static_cast<float>(crop_max_y_),
      static_cast<float>(crop_max_z_), 1.0F));
    crop.filter(*cropped);

    if (publish_debug_clouds_) {
      publish_cloud(cropped, transformed_msg.header, roi_pub_);
    }

    Cloud::Ptr filtered = cropped;
    if (enable_voxel_grid_ && !cropped->empty()) {
      Cloud::Ptr downsampled(new Cloud);
      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(cropped);
      const float leaf = static_cast<float>(voxel_leaf_size_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*downsampled);
      filtered = downsampled;
    }

    Cloud::Ptr result = remove_table_plane(filtered, transformed_msg.header);
    publish_cloud(result, transformed_msg.header, output_pub_);
  }

  Cloud::Ptr remove_table_plane(
    const Cloud::Ptr & cloud, const std_msgs::msg::Header & header)
  {
    if (!remove_table_ || cloud->size() < static_cast<std::size_t>(minimum_plane_inliers_)) {
      return cloud;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(plane_max_angle_deg_ * kPi / 180.0);
    segmentation.setDistanceThreshold(plane_distance_threshold_);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.size() < static_cast<std::size_t>(minimum_plane_inliers_)) {
      RCLCPP_DEBUG(get_logger(), "No table plane passed the minimum inlier count");
      return cloud;
    }

    if (check_table_height_ && !plane_height_matches(*coefficients)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Detected horizontal plane is outside the expected table-height range; keeping it");
      return cloud;
    }

    if (publish_debug_clouds_) {
      Cloud::Ptr table(new Cloud);
      pcl::copyPointCloud(*cloud, inliers->indices, *table);
      publish_cloud(table, header, table_pub_);
    }

    Cloud::Ptr objects(new Cloud);
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*objects);
    return objects;
  }

  bool plane_height_matches(const pcl::ModelCoefficients & coefficients) const
  {
    if (coefficients.values.size() < 4) {
      return false;
    }
    const double a = coefficients.values[0];
    const double b = coefficients.values[1];
    const double c = coefficients.values[2];
    const double d = coefficients.values[3];
    const double normal_length = std::sqrt(a * a + b * b + c * c);
    if (normal_length <= 1e-9 || std::abs(c) <= 1e-9) {
      return false;
    }
    const double height_at_origin = -d / c;
    return std::abs(height_at_origin - expected_table_height_) <= table_height_tolerance_;
  }

  void publish_cloud(
    const Cloud::Ptr & cloud, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    output.header.frame_id = target_frame_;
    publisher->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string roi_topic_;
  std::string table_topic_;
  double transform_timeout_sec_;
  double crop_min_x_;
  double crop_max_x_;
  double crop_min_y_;
  double crop_max_y_;
  double crop_min_z_;
  double crop_max_z_;
  bool enable_voxel_grid_;
  double voxel_leaf_size_;
  bool remove_table_;
  double plane_distance_threshold_;
  double plane_max_angle_deg_;
  int minimum_plane_inliers_;
  bool check_table_height_;
  double expected_table_height_;
  double table_height_tolerance_;
  bool publish_debug_clouds_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PointCloudPreprocessor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_preprocessor"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
