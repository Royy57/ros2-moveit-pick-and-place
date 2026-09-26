/**
 * @file pick_place_task.cpp
 * @brief Construction of the MoveIt Task Constructor (MTC) pick and place task.
 *
 * @author Addison Sears-Collins
 * Modified by: Souvik Roy <sroyy57@gmail.com> (2026) — split out of mtc_node.cpp
 */

#include "pnp_cobot_mtc_pick_place_demo/pick_place_task.h"

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace pick_place
{

namespace mtc = moveit::task_constructor;

namespace
{

/**
 * @brief Planners shared by the stages of the task.
 */
struct Planners
{
  std::shared_ptr<mtc::solvers::PipelinePlanner> arm;                 // OMPL, for free-space arm motion
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> gripper;   // simple joint moves
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian;             // straight-line tool motion
};

/**
 * @brief Convert [x, y, z, roll, pitch, yaw] to a transform.
 */
Eigen::Isometry3d vectorToEigen(const std::vector<double> & values)
{
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

/**
 * @brief Convert [x, y, z, roll, pitch, yaw] to a pose message.
 */
geometry_msgs::msg::Pose vectorToPose(const std::vector<double> & values)
{
  return tf2::toMsg(vectorToEigen(values));
}

/**
 * @brief A direction along the z axis of a frame.
 */
geometry_msgs::msg::Vector3Stamped zDirection(const std::string & frame, double z)
{
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = frame;
  vec.vector.z = z;
  return vec;
}

/**
 * @brief Make the stage's trajectory run on the configured controllers.
 */
void useControllers(mtc::Stage & stage, const PickPlaceParams & p)
{
  stage.properties().set("trajectory_execution_info",
    mtc::TrajectoryExecutionInfo().set__controller_names(p.controller_names));
}

Planners createPlanners(const rclcpp::Node::SharedPtr & node, const PickPlaceParams & p)
{
  Planners planners;

  // MTC on Humble takes a single pipeline name; the planner id is set separately
  planners.arm = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
  planners.arm->setPlannerId("RRTConnectkConfigDefault");

  // Fast, but only for simple motions such as opening and closing the gripper
  planners.gripper = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  planners.cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  planners.cartesian->setMaxVelocityScalingFactor(p.cartesian_max_velocity_scaling);
  planners.cartesian->setMaxAccelerationScalingFactor(p.cartesian_max_acceleration_scaling);
  planners.cartesian->setStepSize(p.cartesian_step_size);

  RCLCPP_INFO(node->get_logger(), "Created OMPL (arm), joint interpolation (gripper) and "
              "Cartesian planners");
  return planners;
}

std::unique_ptr<mtc::stages::MoveTo> moveGripper(
  const std::string & name, const std::string & goal,
  const Planners & planners, const PickPlaceParams & p)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planners.gripper);
  stage->setGroup(p.gripper_group_name);
  stage->setGoal(goal);
  useControllers(*stage, p);
  return stage;
}

std::unique_ptr<mtc::stages::Connect> connect(
  const std::string & name, double timeout,
  const Planners & planners, const PickPlaceParams & p)
{
  auto stage = std::make_unique<mtc::stages::Connect>(
    name,
    mtc::stages::Connect::GroupPlannerVector{
      {p.arm_group_name, planners.arm},
      {p.gripper_group_name, planners.gripper}
    });
  stage->setTimeout(timeout);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  return stage;
}

/**
 * @brief A serial container that inherits the task's eef, group and ik_frame properties.
 */
std::unique_ptr<mtc::SerialContainer> container(const std::string & name, mtc::Task & task)
{
  auto stages = std::make_unique<mtc::SerialContainer>(name);
  task.properties().exposeTo(stages->properties(), {"eef", "group", "ik_frame"});
  stages->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
  return stages;
}

/**
 * @brief Add the "pick object" container: approach, grasp, attach and lift.
 *
 * @param current_state Stage whose state the grasp pose generator checks its poses against
 * @return The "attach object" stage, which the place pose generator follows
 */
mtc::Stage * addPickStages(
  mtc::Task & task, const PickPlaceParams & p, const Planners & planners,
  const std::string & support_surface_id, mtc::Stage * current_state)
{
  auto grasp = container("pick object", task);

  // Approach: move the gripper along its z axis towards the object in a straight line
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", planners.cartesian);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", p.gripper_frame);
    useControllers(*stage, p);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(p.approach_object_min_dist, p.approach_object_max_dist);
    stage->setDirection(zDirection(p.gripper_frame, p.approach_object_direction_z));
    grasp->insert(std::move(stage));
  }

  // Grasp pose: sample grasps around the object's z axis, and solve IK for each
  {
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose(p.gripper_open_pose);
    stage->setObject(p.object_name);
    stage->setAngleDelta(p.grasp_pose_angle_delta);
    stage->setMonitoredStage(current_state);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    ik->setMaxIKSolutions(p.grasp_pose_max_ik_solutions);
    ik->setMinSolutionDistance(p.grasp_pose_min_solution_distance);
    ik->setIKFrame(vectorToEigen(p.grasp_frame_transform), p.gripper_frame);  // tool center point
    ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    grasp->insert(std::move(ik));
  }

  // Let the gripper touch the object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
    stage->allowCollisions(
      p.object_name,
      task.getRobotModel()->getJointModelGroup(p.gripper_group_name)
        ->getLinkModelNamesWithCollisionGeometry(),
      true);
    grasp->insert(std::move(stage));
  }

  grasp->insert(moveGripper("close gripper", p.gripper_close_pose, planners, p));

  // The object may touch the table until it is lifted
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow collision (object,support_surface)");
    stage->allowCollisions({p.object_name}, {support_surface_id}, true);
    grasp->insert(std::move(stage));
  }

  mtc::Stage * attach_object_stage = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(p.object_name, p.gripper_frame);
    attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
  }

  // Lift straight up in the world frame
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", planners.cartesian);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(p.lift_object_min_dist, p.lift_object_max_dist);
    stage->setIKFrame(p.gripper_frame);
    stage->properties().set("marker_ns", "lift_object");
    useControllers(*stage, p);
    stage->setDirection(zDirection(p.world_frame, p.lift_object_direction_z));
    grasp->insert(std::move(stage));
  }

  // Once lifted, the object must not hit the table again
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "forbid collision (object,support_surface)");
    stage->allowCollisions({p.object_name}, {support_surface_id}, false);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  return attach_object_stage;
}

/**
 * @brief Add the "place object" container: lower, release, detach and retreat.
 *
 * @param attach_object_stage Stage whose successful pick solutions the place pose follows
 */
void addPlaceStages(
  mtc::Task & task, const PickPlaceParams & p, const Planners & planners,
  mtc::Stage * attach_object_stage)
{
  auto place = container("place object", task);

  // Lower straight down in the world frame
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", planners.cartesian);
    stage->properties().set("marker_ns", "lower_object");
    stage->properties().set("link", p.gripper_frame);
    useControllers(*stage, p);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(p.lower_object_min_dist, p.lower_object_max_dist);
    stage->setDirection(zDirection(p.world_frame, p.lower_object_direction_z));
    place->insert(std::move(stage));
  }

  // Place pose: the object's centre above place_pose, with IK solved for the gripper
  {
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject(p.object_name);

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = p.world_frame;
    target_pose.pose = vectorToPose(p.place_pose);
    target_pose.pose.position.z += p.place_pose_z_offset_factor * p.object_dimensions[0];
    stage->setPose(target_pose);
    stage->setMonitoredStage(attach_object_stage);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    ik->setMaxIKSolutions(p.place_pose_max_ik_solutions);
    ik->setIKFrame(vectorToEigen(p.grasp_frame_transform), p.gripper_frame);  // tool center point
    ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    place->insert(std::move(ik));
  }

  place->insert(moveGripper("open gripper", p.gripper_open_pose, planners, p));

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
    stage->allowCollisions(
      p.object_name, *task.getRobotModel()->getJointModelGroup(p.gripper_group_name), false);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(p.object_name, p.gripper_frame);
    place->insert(std::move(stage));
  }

  // Retreat: back the gripper away along its z axis
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", planners.cartesian);
    useControllers(*stage, p);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(p.retreat_min_distance, p.retreat_max_distance);
    stage->setIKFrame(p.gripper_frame);
    stage->properties().set("marker_ns", "retreat");
    stage->setDirection(zDirection(p.gripper_frame, p.retreat_direction_z));
    place->insert(std::move(stage));
  }

  task.add(std::move(place));
}

}  // namespace

mtc::Task buildPickPlaceTask(
  const rclcpp::Node::SharedPtr & node,
  const PickPlaceParams & p,
  const std::string & support_surface_id)
{
  RCLCPP_INFO(node->get_logger(), "Creating task for %s '%s' with dimensions [%s]",
              p.object_type.c_str(), p.object_name.c_str(), join(p.object_dimensions).c_str());

  mtc::Task task;
  task.stages()->setName("pick_place_task");
  task.loadRobotModel(node, "robot_description");

  const Planners planners = createPlanners(node, p);

  task.setProperty("trajectory_execution_info",
    mtc::TrajectoryExecutionInfo().set__controller_names(p.controller_names));
  task.setProperty("group", p.arm_group_name);     // main planning group
  task.setProperty("eef", p.gripper_group_name);   // end-effector group
  task.setProperty("ik_frame", p.gripper_frame);   // frame for inverse kinematics

  auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
  mtc::Stage * current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  task.add(moveGripper("open gripper", p.gripper_open_pose, planners, p));
  task.add(connect("move to pick", p.move_to_pick_timeout, planners, p));

  mtc::Stage * attach_object_stage =
    addPickStages(task, p, planners, support_surface_id, current_state_ptr);

  task.add(connect("move to place", p.move_to_place_timeout, planners, p));
  addPlaceStages(task, p, planners, attach_object_stage);

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", planners.arm);
    useControllers(*stage, p);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(p.arm_home_pose);
    task.add(std::move(stage));
  }

  return task;
}

}  // namespace pick_place
