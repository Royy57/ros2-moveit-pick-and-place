/**
 * @file mtc_node.cpp
 * @brief MoveIt Task Constructor (MTC) node for a perception-based pick and place task.
 *
 * The node:
 * 1. Asks the perception server (GetPlanningScene service) for the objects on the table
 * 2. Adds them to the MoveIt planning scene and targets the object perception selected
 * 3. Builds the pick and place task (see pick_place_task.h) and plans it
 * 4. Executes the best solution if the `execute` parameter is true
 *
 * Parameters are declared in pick_place_params.cpp and set in config/mtc_node_params.yaml.
 *
 * @author Addison Sears-Collins
 * Modified by: Souvik Roy <sroyy57@gmail.com> (2026) — ported to ROS 2 Humble / Gazebo Fortress
 * @date December 20, 2024
 */

#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "pnp_cobot_mtc_pick_place_demo/get_planning_scene_client.h"
#include "pnp_cobot_mtc_pick_place_demo/pick_place_params.h"
#include "pnp_cobot_mtc_pick_place_demo/pick_place_task.h"

namespace mtc = moveit::task_constructor;

/**
 * @brief Node that sets up the planning scene from perception and runs the pick and place task.
 */
class MTCTaskNode : public rclcpp::Node
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions & options);

  /// Get the objects from the perception server and add them to the planning scene.
  void setupPlanningScene();

  /// Build, plan and (if `execute` is set) execute the pick and place task.
  void doTask();

private:
  /// Point the object_* parameters at the object perception selected as the target.
  void updateObjectParameters(const moveit_msgs::msg::CollisionObject & collision_object);

  std::shared_ptr<GetPlanningSceneClient> planning_scene_client_;
  mtc::Task task_;  // kept alive so its solution stays available for RViz
  std::string target_object_id_;
  std::string support_surface_id_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions & options)
: Node("mtc_node", options)
{
  pick_place::declareParameters(*this);
  RCLCPP_INFO(this->get_logger(), "All parameters have been declared with descriptions");

  planning_scene_client_ = std::make_shared<GetPlanningSceneClient>();
}

void MTCTaskNode::updateObjectParameters(const moveit_msgs::msg::CollisionObject & collision_object)
{
  this->set_parameter(rclcpp::Parameter("object_name", collision_object.id));
  RCLCPP_INFO(this->get_logger(), "Updated object_name: '%s'", collision_object.id.c_str());

  if (collision_object.primitives.empty()) {
    return;
  }

  using shape_msgs::msg::SolidPrimitive;
  const auto & primitive = collision_object.primitives[0];
  std::string object_type;
  std::vector<double> dimensions;
  if (primitive.type == SolidPrimitive::CYLINDER) {
    object_type = "cylinder";
    dimensions = {primitive.dimensions[SolidPrimitive::CYLINDER_HEIGHT],
                  primitive.dimensions[SolidPrimitive::CYLINDER_RADIUS]};
  } else if (primitive.type == SolidPrimitive::BOX) {
    object_type = "box";
    dimensions = {primitive.dimensions[SolidPrimitive::BOX_X],
                  primitive.dimensions[SolidPrimitive::BOX_Y],
                  primitive.dimensions[SolidPrimitive::BOX_Z]};
  } else {
    RCLCPP_WARN(this->get_logger(), "Unsupported object type. Only cylinders and boxes are supported.");
    return;
  }

  this->set_parameter(rclcpp::Parameter("object_type", object_type));
  this->set_parameter(rclcpp::Parameter("object_dimensions", dimensions));
  RCLCPP_INFO(this->get_logger(), "Updated object_type: '%s'", object_type.c_str());
  RCLCPP_INFO(this->get_logger(), "Updated object_dimensions: [%s]",
              pick_place::join(dimensions).c_str());
}

void MTCTaskNode::setupPlanningScene()
{
  const auto params = pick_place::loadParameters(*this);
  RCLCPP_INFO(this->get_logger(), "Initial target object: %s '%s' [%s] in frame %s",
              params.object_type.c_str(), params.object_name.c_str(),
              pick_place::join(params.object_dimensions).c_str(),
              params.object_reference_frame.c_str());

  RCLCPP_INFO(this->get_logger(), "Sending GetPlanningScene service request...");
  auto response = planning_scene_client_->call_service(params.object_type, params.object_dimensions);
  RCLCPP_INFO(this->get_logger(), "Service call to the GetPlanningScene service completed.");
  if (!response.success) {
    RCLCPP_WARN(this->get_logger(), "The perception server reported that it did not succeed");
  }

  target_object_id_ = response.target_object_id;
  support_surface_id_ = response.support_surface_id;
  const auto & collision_objects = response.scene_world.collision_objects;

  RCLCPP_INFO(this->get_logger(), "Applying collision objects from service response...");
  moveit::planning_interface::PlanningSceneInterface psi;
  if (!psi.applyCollisionObjects(collision_objects)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add collision objects from service response");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Successfully added %zu collision objects from service response to the planning scene",
                collision_objects.size());
  }

  RCLCPP_INFO(this->get_logger(), "Received target_object_id from service: '%s'",
              target_object_id_.c_str());
  for (const auto & collision_object : collision_objects) {
    if (collision_object.id == target_object_id_) {
      updateObjectParameters(collision_object);
      break;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Planning scene setup completed");
}

void MTCTaskNode::doTask()
{
  RCLCPP_INFO(this->get_logger(), "Starting the pick and place task");

  // Read the parameters now: setupPlanningScene() updated the object ones
  const auto params = pick_place::loadParameters(*this);
  task_ = pick_place::buildPickPlaceTask(shared_from_this(), params, support_surface_id_);

  try {
    task_.init();
    RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
  } catch (mtc::InitStageException & e) {
    RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    return;
  }

  if (!task_.plan(params.max_solutions)) {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Task planning succeeded");

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(this->get_logger(), "Published solution for visualization");

  if (!params.execute) {
    RCLCPP_INFO(this->get_logger(), "Execution skipped as per configuration");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Executing the planned task");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Task executed successfully");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;

  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_task_node);

    try {
      RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
      mtc_task_node->setupPlanningScene();
      RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
      mtc_task_node->doTask();
      RCLCPP_INFO(mtc_task_node->get_logger(),
                  "Task execution completed. Keeping node alive for visualization. Press Ctrl+C to exit.");

      // Keep the node running (and its solution visible in RViz) until Ctrl+C
      executor.spin();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(mtc_task_node->get_logger(), "An error occurred: %s", e.what());
      ret = 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error during node setup: %s", e.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
