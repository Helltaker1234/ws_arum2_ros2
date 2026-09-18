#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/empty.hpp>
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


#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#define TCP_DISTANCE 0.00



static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class DglGraspPoseGenerator : public mtc::stages::GeneratePose
{
public:
  DglGraspPoseGenerator(const std::string& name,
                        std::vector<geometry_msgs::msg::PoseStamped> candidates,
                        std::vector<double> costs,
                        std::string target_object_id,
                        std::vector<std::string> hand_collision_links)
    : GeneratePose(name)
    , candidates_(std::move(candidates))
    , costs_(std::move(costs))
    , target_object_id_(std::move(target_object_id))
    , hand_collision_links_(std::move(hand_collision_links))
  {
    properties().declare<std::string>("eef", "name of end-effector");
  }

  void setPreGraspPose(const moveit_msgs::msg::RobotState& pregrasp)
  {
    pregrasp_ = pregrasp;
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    GeneratePose::init(robot_model);
    if (candidates_.empty())
      throw mtc::InitStageException(*this, "DGL returned no grasp candidates");
    if (candidates_.size() != costs_.size())
      throw mtc::InitStageException(*this, "DGL candidate and cost counts differ");

    const std::string& eef = properties().get<std::string>("eef");
    if (!robot_model->hasEndEffector(eef))
      throw mtc::InitStageException(*this, "unknown end effector: " + eef);
  }

protected:
  void onNewSolution(const mtc::SolutionBase& solution) override
  {
    upstream_solutions_.push(&solution);
  }

  void compute() override
  {
    if (upstream_solutions_.empty())
      return;

    auto scene = upstream_solutions_.pop()->end()->scene()->diff();
    moveit::core::robotStateMsgToRobotState(pregrasp_, scene->getCurrentStateNonConst());
    scene->getAllowedCollisionMatrixNonConst().setEntry(target_object_id_, hand_collision_links_, true);

    for (std::size_t i = 0; i < candidates_.size(); ++i)
    {
      mtc::InterfaceState state(scene);
      state.properties().set("target_pose", candidates_[i]);

      mtc::SubTrajectory trajectory;
      trajectory.setCost(costs_[i]);
      trajectory.setComment("DGL grasp candidate " + std::to_string(i));
      spawn(std::move(state), std::move(trajectory));
    }
  }

private:
  moveit_msgs::msg::RobotState pregrasp_;
  std::vector<geometry_msgs::msg::PoseStamped> candidates_;
  std::vector<double> costs_;
  std::string target_object_id_;
  std::vector<std::string> hand_collision_links_;
};

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

private:
  bool requestGraspCandidates();
  bool prepareTargetObject();

  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  std::vector<geometry_msgs::msg::PoseStamped> grasp_candidates_;
  std::vector<double> grasp_costs_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  if (!node_->has_parameter("target_object_id"))
    node_->declare_parameter<std::string>("target_object_id", "target_object");
  if (!node_->has_parameter("target_size_x"))
    node_->declare_parameter<double>("target_size_x", 0.08);
  if (!node_->has_parameter("target_size_y"))
    node_->declare_parameter<double>("target_size_y", 0.08);
  if (!node_->has_parameter("target_size_z"))
    node_->declare_parameter<double>("target_size_z", 0.12);
  if (!node_->has_parameter("target_offset_x"))
    node_->declare_parameter<double>("target_offset_x", 0.0);
  if (!node_->has_parameter("target_offset_y"))
    node_->declare_parameter<double>("target_offset_y", 0.0);
  if (!node_->has_parameter("target_offset_z"))
    node_->declare_parameter<double>("target_offset_z", 0.0);
  if (!node_->has_parameter("octomap_rebuild_wait_seconds"))
    node_->declare_parameter<double>("octomap_rebuild_wait_seconds", 1.0);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool MTCTaskNode::requestGraspCandidates()
{
  using SampleGraspPoses = dgl_ros_interfaces::action::SampleGraspPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SampleGraspPoses>;

  struct RequestState
  {
    std::mutex mutex;
    std::condition_variable condition;
    bool done{ false };
    bool succeeded{ false };
    std::vector<geometry_msgs::msg::PoseStamped> candidates;
    std::vector<double> costs;
  };

  auto client = rclcpp_action::create_client<SampleGraspPoses>(node_, "/sample_grasp_poses");
  if (!client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "DGL action server '/sample_grasp_poses' is not available");
    return false;
  }

  auto request_state = std::make_shared<RequestState>();
  SampleGraspPoses::Goal goal;
  goal.action_name = "sample_grasp_poses";

  rclcpp_action::Client<SampleGraspPoses>::SendGoalOptions options;
  options.goal_response_callback = [request_state](const GoalHandle::SharedPtr& goal_handle) {
    if (!goal_handle)
    {
      std::lock_guard<std::mutex> lock(request_state->mutex);
      request_state->done = true;
      request_state->condition.notify_one();
    }
  };
  options.feedback_callback =
      [request_state](GoalHandle::SharedPtr, const std::shared_ptr<const SampleGraspPoses::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(request_state->mutex);
        request_state->candidates = feedback->grasp_candidates;
        request_state->costs = feedback->costs;
      };
  options.result_callback = [request_state](const GoalHandle::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(request_state->mutex);
    request_state->succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    request_state->done = true;
    request_state->condition.notify_one();
  };

  client->async_send_goal(goal, options);

  std::unique_lock<std::mutex> lock(request_state->mutex);
  if (!request_state->condition.wait_for(lock, std::chrono::seconds(120),
                                         [&request_state]() { return request_state->done; }))
  {
    RCLCPP_ERROR(LOGGER, "Timed out while waiting for DGL grasp candidates");
    client->async_cancel_all_goals();
    return false;
  }

  if (!request_state->succeeded || request_state->candidates.empty())
  {
    RCLCPP_ERROR(LOGGER, "DGL action did not return any grasp candidates");
    return false;
  }
  if (request_state->candidates.size() != request_state->costs.size())
  {
    RCLCPP_ERROR(LOGGER, "DGL returned %zu candidates but %zu costs",
                 request_state->candidates.size(), request_state->costs.size());
    return false;
  }

  std::vector<std::size_t> priority(request_state->candidates.size());
  for (std::size_t i = 0; i < priority.size(); ++i)
    priority[i] = i;
  std::stable_sort(priority.begin(), priority.end(), [&request_state](std::size_t lhs, std::size_t rhs) {
    return request_state->costs[lhs] < request_state->costs[rhs];
  });

  grasp_candidates_.clear();
  grasp_costs_.clear();
  grasp_candidates_.reserve(priority.size());
  grasp_costs_.reserve(priority.size());
  for (const std::size_t index : priority)
  {
    grasp_candidates_.push_back(request_state->candidates[index]);
    grasp_costs_.push_back(request_state->costs[index]);
  }
  RCLCPP_INFO(LOGGER, "Received %zu grasp candidates from DGL", grasp_candidates_.size());
  return true;
}

bool MTCTaskNode::prepareTargetObject()
{
  if (grasp_candidates_.empty())
    return false;

  const double size_x = node_->get_parameter("target_size_x").as_double();
  const double size_y = node_->get_parameter("target_size_y").as_double();
  const double size_z = node_->get_parameter("target_size_z").as_double();
  const double offset_x = node_->get_parameter("target_offset_x").as_double();
  const double offset_y = node_->get_parameter("target_offset_y").as_double();
  const double offset_z = node_->get_parameter("target_offset_z").as_double();
  if (size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0)
  {
    RCLCPP_ERROR(LOGGER, "target_size_x, target_size_y, and target_size_z must be positive");
    return false;
  }

  const std::string& frame_id = grasp_candidates_.front().header.frame_id;
  if (frame_id.empty() || std::any_of(grasp_candidates_.begin(), grasp_candidates_.end(),
                                      [&frame_id](const auto& candidate) {
                                        return candidate.header.frame_id != frame_id;
                                      }))
  {
    RCLCPP_ERROR(LOGGER, "All DGL grasp candidates must use the same non-empty frame");
    return false;
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = node_->get_parameter("target_object_id").as_string();
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = { size_x, size_y, size_z };



  geometry_msgs::msg::Pose pose;
  for (const auto& candidate : grasp_candidates_)
  {
    pose.position.x += candidate.pose.position.x;
    pose.position.y += candidate.pose.position.y;
    pose.position.z += candidate.pose.position.z;
  }
  const double candidate_count = static_cast<double>(grasp_candidates_.size());
  pose.position.x = pose.position.x / candidate_count + offset_x;
  pose.position.y = pose.position.y / candidate_count + offset_y;
  pose.position.z = pose.position.z / candidate_count + offset_z;
  pose.orientation.w = 1.0;

  object.primitives.push_back(std::move(primitive));
  object.primitive_poses.push_back(pose);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.header.frame_id = frame_id;
  remove_object.id = object.id;
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  if (!planning_scene_interface.applyCollisionObjects({ remove_object, object }))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add target CollisionObject '%s'", object.id.c_str());
    return false;
  }

  auto clear_octomap = node_->create_client<std_srvs::srv::Empty>("/clear_octomap");
  if (!clear_octomap->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "Service '/clear_octomap' is not available");
    return false;
  }

  auto future = clear_octomap->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "Timed out while clearing the OctoMap");
    return false;
  }

  const double rebuild_wait = node_->get_parameter("octomap_rebuild_wait_seconds").as_double();
  if (rebuild_wait > 0.0)
    rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(rebuild_wait)));

  RCLCPP_INFO(LOGGER,
              "Registered target '%s' in frame '%s' at [%.3f, %.3f, %.3f], cleared OctoMap, and waited %.2f s",
              object.id.c_str(), frame_id.c_str(), pose.position.x, pose.position.y, pose.position.z, rebuild_wait);
  return true;
}

void MTCTaskNode::doTask()
{
  if (!requestGraspCandidates())
    return;
  if (!prepareTargetObject())
    return;

  try
  {
    task_ = createTask();
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Failed to create task: %s", e.what());
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
  const std::string target_object_id = node_->get_parameter("target_object_id").as_string();

  const std::string world_frame = "world";

  constexpr double pi = 3.14159265358979323846;

  const std::map<std::string, double> open_gripper = {
    { "gripper_left_finger_joint_L", 0.03 },
  };
  const std::map<std::string, double> closed_gripper = {
    { "gripper_left_finger_joint_L", 0.0 },
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

  {
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision with target");
    stage->allowCollisions(target_object_id,
                            *task.getRobotModel()->getJointModelGroup(hand_group_name), true);
    task.add(std::move(stage));
  }

  const std::map<std::string, double> left_home_pose = {
    { "shoulder_pitch_joint_L", 77.0 * pi / 180.0 },
    { "elbow_joint_L", 112.0 * pi / 180.0 },
    { "wrist_pitch_joint_L", 38.0 * pi / 180.0 },
  };

  const std::map<std::string, double> right_home_pose = {
    { "shoulder_pitch_joint_R", 77.0 * pi / 180.0 },
    { "elbow_joint_R", 112.0 * pi / 180.0 },
    { "wrist_pitch_joint_R", 38.0 * pi / 180.0 },
  };

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move right arm to home", interpolation_planner);
    stage->setGroup("areumii_arm_r");
    stage->setGoal(right_home_pose);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move left arm to home", interpolation_planner);
    stage->setGroup("areumii_arm_l");
    stage->setGoal(left_home_pose);
    task.add(std::move(stage));
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

  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick highest-priority DGL grasp");
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", "approach_object");
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(TCP_DISTANCE, 0.15);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = -1.0; // gripper_hand_link_L_1 의 -z 방향이 EE 의 approch 방향임
      stage->setDirection(direction);
      pick->insert(std::move(stage));
    }

    {
      auto generator = std::make_unique<DglGraspPoseGenerator>(
          "generate deep grasp poses", grasp_candidates_, grasp_costs_, target_object_id,
          task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry());
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "grasp_pose");
      moveit_msgs::msg::RobotState pregrasp_state;
      pregrasp_state.is_diff = true;
      pregrasp_state.joint_state.name = { "gripper_left_finger_joint_L" };
      pregrasp_state.joint_state.position = { open_gripper.at("gripper_left_finger_joint_L") };
      generator->setPreGraspPose(pregrasp_state);
      generator->setMonitoredStage(open_hand_stage);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();

      // GPD 가 생성한 EE Pose 좌표계에 맞추기 위한 회전 변환.
      grasp_frame_transform.linear() =
      (Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(-pi / 2.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

      grasp_frame_transform.translation().z() = -TCP_DISTANCE; // gripper_hand_link_L_1 원점에서 -Z 방향으로 6cm 떨어진 지점을 파지점으로 설정 >> 이거 어짜피 TCP Frame 정의하면 0으로 변할 듯.

      auto stage = std::make_unique<mtc::stages::ComputeIK>("compute grasp IK", std::move(generator));
      stage->setMaxIKSolutions(8);
      stage->setMinSolutionDistance(1.0);
      stage->setIKFrame(grasp_frame_transform, hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      pick->insert(std::move(stage));
    }


    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(closed_gripper);
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach target");
      stage->attachObject(target_object_id, hand_frame);
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

    task.add(std::move(pick));
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

  mtc_task_node->doTask();

  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}
