#include "moveit/move_group_interface/move_group_interface.h"


int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);



  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );


  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "areum2_arm_l"/*"manipulator"*/);


  // move_group_interface 객체 생성 후
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  RCLCPP_INFO(node->get_logger(), "Current Pose Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z);
}
