#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif


#include <map>
#include <string>



static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  constexpr double table_height = 0.04;
  constexpr double table_top_z = 0.34;
  constexpr double object_height = 0.10;

  // The left shoulder is at approximately (0.038, 0.079, 0.697) in the URDF.
  // Place the support and object in front of, and slightly to the left of, that shoulder.
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 0.3, 0.3, table_height };
  table.pose.orientation.w = 1.0;
  table.pose.position.x = 0.30;
  table.pose.position.y = 0.20;
  table.pose.position.z = table_top_z - table_height / 2.0;

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "object";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { object_height, 0.025 };
  object.pose.orientation.w = 1.0;
  object.pose.position.x = 0.20;
  object.pose.position.y = 0.15;
  object.pose.position.z = table_top_z + object_height / 2.0;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  if (!planning_scene_interface.applyCollisionObjects({ table, object }))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add table and object to the planning scene");
    return;
  }

  RCLCPP_INFO(
      LOGGER, "Added object at (%.2f, %.2f, %.2f) on table '%s'",
      object.pose.position.x, object.pose.position.y, object.pose.position.z, table.id.c_str());
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Areum II left gripper task");
  task.loadRobotModel(node_);

  const std::string arm_group_name = "areumii_arm_l";
  const std::string hand_group_name = "areumii_gripper_l";
  const std::string eef_name = "gripper_l";
  const std::string hand_frame = "gripper_hand_link_L_1";

  const std::string world_frame = "world";

  const std::string object_name = "object";
  const std::string support_name = "table";

  constexpr double pi = 3.14159265358979323846;

  const std::map<std::string, double> open_gripper = {
    { "gripper_left_finger_joint_L", 0.03 },
  };
  const std::map<std::string, double> closed_gripper = {
    { "gripper_left_finger_joint_L", 0.0 },
  };
  const std::map<std::string, double> home_pose = {
    { "shoulder_pitch_joint_L", 0.0 },
    { "shoulder_roll_joint_L", 0.0 },
    { "shouler_yaw_joint_L", 0.0 },
    { "elbow_joint_L", 0.0 },
    { "wrist_roll_joint_L", 0.0 },
    { "wrist_yaw_joint_L", 0.0 },
    { "wrist_pitch_joint_L", 0.0 },
  };

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  // Set planner
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);

  // CurrentState + PredicateFilter: picking is applicable only when the object is detached.
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    auto filter = std::make_unique<mtc::stages::PredicateFilter>( "object is not attached", std::move(current_state));

    filter->setPredicate(
        [object_name](const mtc::SolutionBase& solution, std::string& comment) {
          if (solution.start()->scene()->getCurrentState().hasAttachedBody(object_name))
          {
            comment = "object '" + object_name + "' is already attached";
            return false;
          }
          return true;
        }
      );

    task.add(std::move(filter));
  }

  mtc::Stage* open_hand_stage = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal(open_gripper);
    open_hand_stage = stage.get();
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>("move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  mtc::Stage* pick_stage = nullptr;
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", "approach_object");
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(0.05, 0.15);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      pick->insert(std::move(stage));
    }

    {
      auto generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "grasp_pose");
      moveit_msgs::msg::RobotState pregrasp_state;
      pregrasp_state.is_diff = true;
      pregrasp_state.joint_state.name = { "gripper_left_finger_joint_L" };
      pregrasp_state.joint_state.position = { open_gripper.at("gripper_left_finger_joint_L") };
      generator->setPreGraspPose(pregrasp_state);
      generator->setObject(object_name);
      generator->setAngleDelta(pi / 12.0);
      generator->setMonitoredStage(open_hand_stage);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.linear() =
          (Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ()))
              .matrix();
      grasp_frame_transform.translation().z() = 0.1;

      auto stage = std::make_unique<mtc::stages::ComputeIK>("compute grasp IK", std::move(generator));
      stage->setMaxIKSolutions(8);
      stage->setMinSolutionDistance(1.0);
      stage->setIKFrame(grasp_frame_transform, hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, object)");
      stage->allowCollisions(
          object_name,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(closed_gripper);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object, support)");
      stage->allowCollisions({ object_name }, { support_name }, true);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", "lift_object");
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(0.10, 0.25);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = world_frame;
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object, support)");
      stage->allowCollisions({ object_name }, { support_name }, false);
      pick->insert(std::move(stage));
    }

    pick_stage = pick.get();
    task.add(std::move(pick));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>("move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(
        mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", "lower_object");
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(0.03, 0.13);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = world_frame;
      direction.vector.z = -1.0;
      stage->setDirection(direction);
      place->insert(std::move(stage));
    }

    {
      auto generator = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      generator->properties().set("marker_ns", "place_pose");
      generator->setObject(object_name);
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = world_frame;
      target_pose.pose.position.x = 0.5;
      target_pose.pose.position.y = 0.3;
      target_pose.pose.position.z = 0.1;
      target_pose.pose.orientation.w = 1.0;
      generator->setPose(target_pose);
      generator->setMonitoredStage(pick_stage);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.linear() =
          (Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ()))
              .matrix();
      grasp_frame_transform.translation().z() = 0.1;

      auto stage = std::make_unique<mtc::stages::ComputeIK>("compute place IK", std::move(generator));
      stage->setMaxIKSolutions(2);
      stage->setMinSolutionDistance(1.0);
      stage->setIKFrame(grasp_frame_transform, hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(open_gripper);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand, object)");
      stage->allowCollisions(
          object_name,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", "retreat");
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(0.10, 0.25);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = -1.0;
      stage->setDirection(direction);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(home_pose);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
