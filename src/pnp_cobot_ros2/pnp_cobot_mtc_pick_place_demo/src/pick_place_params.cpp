/**
 * @file pick_place_params.cpp
 * @brief Declaration and loading of the MTC pick and place task parameters.
 *
 * @author Addison Sears-Collins
 * Modified by: Souvik Roy <sroyy57@gmail.com> (2026) — split out of mtc_node.cpp
 */

#include "pnp_cobot_mtc_pick_place_demo/pick_place_params.h"

#include <sstream>

namespace pick_place
{

namespace
{
/**
 * @brief Declare one parameter with a description, unless it was already declared from
 * parameter overrides (the node is created with automatically_declare_parameters_from_overrides).
 */
template<typename T>
void declare(rclcpp::Node & node, const std::string & name, const T & default_value,
             const std::string & description)
{
  if (node.has_parameter(name)) {
    return;
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  node.declare_parameter(name, default_value, descriptor);
}
}  // namespace

void declareParameters(rclcpp::Node & node)
{
  using Strings = std::vector<std::string>;
  using Doubles = std::vector<double>;

  // General parameters
  declare(node, "execute", false, "Whether to execute the planned task");
  declare(node, "max_solutions", 25, "Maximum number of solutions to compute");
  declare(node, "controller_names", Strings{"arm_controller", "gripper_action_controller"},
          "Names of the controllers to use");

  // Robot configuration parameters
  declare(node, "arm_group_name", "arm", "Name of the arm group in the SRDF");
  declare(node, "gripper_group_name", "gripper", "Name of the gripper group in the SRDF");
  declare(node, "gripper_frame", "link6_flange", "Name of the gripper frame");
  declare(node, "gripper_open_pose", "open", "Name of the gripper open pose");
  declare(node, "gripper_close_pose", "half_closed", "Name of the gripper closed pose");
  declare(node, "arm_home_pose", "home", "Name of the arm home pose");
  declare(node, "world_frame", "base_link", "Name of the world frame");

  // Object parameters
  declare(node, "object_name", "object", "Name of the object to be manipulated");
  declare(node, "object_type", "cylinder", "Type of the object to be manipulated");
  declare(node, "object_reference_frame", "base_link", "Reference frame for the object");
  declare(node, "object_dimensions", Doubles{0.35, 0.0125},
          "Dimensions of the object [height, radius]");
  declare(node, "object_pose", Doubles{0.22, 0.12, 0.0, 0.0, 0.0, 0.0},
          "Initial pose of the object [x, y, z, roll, pitch, yaw]");

  // Grasp and place parameters
  declare(node, "grasp_frame_transform", Doubles{0.0, 0.0, 0.096, 1.5708, 0.0, 0.0},
          "Transform from gripper frame to grasp frame [x, y, z, roll, pitch, yaw]");
  declare(node, "place_pose", Doubles{-0.183, -0.14, 0.0, 0.0, 0.0, 0.0},
          "Pose where the object should be placed [x, y, z, roll, pitch, yaw]");
  declare(node, "place_pose_z_offset_factor", 0.5,
          "Factor to multiply object height for place pose Z offset");

  // Motion planning parameters
  declare(node, "approach_object_min_dist", 0.0015, "Minimum approach distance to the object");
  declare(node, "approach_object_max_dist", 0.3, "Maximum approach distance to the object");
  declare(node, "lift_object_min_dist", 0.005, "Minimum lift distance for the object");
  declare(node, "lift_object_max_dist", 0.3, "Maximum lift distance for the object");
  declare(node, "lower_object_min_dist", 0.005, "Minimum distance for lowering object");
  declare(node, "lower_object_max_dist", 0.4, "Maximum distance for lowering object");
  declare(node, "retreat_min_distance", 0.025, "Minimum distance for retreat motion");
  declare(node, "retreat_max_distance", 0.25, "Maximum distance for retreat motion");

  // Direction vector parameters
  declare(node, "approach_object_direction_z", 1.0,
          "Z component of approach object direction vector");
  declare(node, "lift_object_direction_z", 1.0, "Z component of lift object direction vector");
  declare(node, "lower_object_direction_z", -1.0, "Z component of lower object direction vector");
  declare(node, "retreat_direction_z", -1.0, "Z component of retreat direction vector");

  // Timeout parameters
  declare(node, "move_to_pick_timeout", 10.0, "Timeout for move to pick stage (seconds)");
  declare(node, "move_to_place_timeout", 10.0, "Timeout for move to place stage (seconds)");

  // Grasp and place pose generation parameters
  declare(node, "grasp_pose_angle_delta", 0.1309,
          "Angular resolution for sampling grasp poses (radians)");
  declare(node, "grasp_pose_max_ik_solutions", 10,
          "Maximum number of IK solutions for grasp pose generation");
  declare(node, "grasp_pose_min_solution_distance", 0.8,
          "Minimum distance in joint-space units between IK solutions for grasp pose");
  declare(node, "place_pose_max_ik_solutions", 10,
          "Maximum number of IK solutions for place pose generation");

  // Cartesian planner parameters
  declare(node, "cartesian_max_velocity_scaling", 1.0,
          "Max velocity scaling factor for Cartesian planner");
  declare(node, "cartesian_max_acceleration_scaling", 1.0,
          "Max acceleration scaling factor for Cartesian planner");
  declare(node, "cartesian_step_size", 0.00025, "Step size for Cartesian planner");
}

PickPlaceParams loadParameters(const rclcpp::Node & node)
{
  auto get = [&node](const std::string & name) { return node.get_parameter(name); };

  PickPlaceParams p;
  p.execute = get("execute").as_bool();
  p.max_solutions = static_cast<int>(get("max_solutions").as_int());
  p.controller_names = get("controller_names").as_string_array();

  p.arm_group_name = get("arm_group_name").as_string();
  p.gripper_group_name = get("gripper_group_name").as_string();
  p.gripper_frame = get("gripper_frame").as_string();
  p.gripper_open_pose = get("gripper_open_pose").as_string();
  p.gripper_close_pose = get("gripper_close_pose").as_string();
  p.arm_home_pose = get("arm_home_pose").as_string();
  p.world_frame = get("world_frame").as_string();

  p.object_name = get("object_name").as_string();
  p.object_type = get("object_type").as_string();
  p.object_reference_frame = get("object_reference_frame").as_string();
  p.object_dimensions = get("object_dimensions").as_double_array();
  p.object_pose = get("object_pose").as_double_array();

  p.grasp_frame_transform = get("grasp_frame_transform").as_double_array();
  p.place_pose = get("place_pose").as_double_array();
  p.place_pose_z_offset_factor = get("place_pose_z_offset_factor").as_double();

  p.approach_object_min_dist = get("approach_object_min_dist").as_double();
  p.approach_object_max_dist = get("approach_object_max_dist").as_double();
  p.lift_object_min_dist = get("lift_object_min_dist").as_double();
  p.lift_object_max_dist = get("lift_object_max_dist").as_double();
  p.lower_object_min_dist = get("lower_object_min_dist").as_double();
  p.lower_object_max_dist = get("lower_object_max_dist").as_double();
  p.retreat_min_distance = get("retreat_min_distance").as_double();
  p.retreat_max_distance = get("retreat_max_distance").as_double();

  p.approach_object_direction_z = get("approach_object_direction_z").as_double();
  p.lift_object_direction_z = get("lift_object_direction_z").as_double();
  p.lower_object_direction_z = get("lower_object_direction_z").as_double();
  p.retreat_direction_z = get("retreat_direction_z").as_double();

  p.move_to_pick_timeout = get("move_to_pick_timeout").as_double();
  p.move_to_place_timeout = get("move_to_place_timeout").as_double();

  p.grasp_pose_angle_delta = get("grasp_pose_angle_delta").as_double();
  p.grasp_pose_max_ik_solutions = static_cast<int>(get("grasp_pose_max_ik_solutions").as_int());
  p.grasp_pose_min_solution_distance = get("grasp_pose_min_solution_distance").as_double();
  p.place_pose_max_ik_solutions = static_cast<int>(get("place_pose_max_ik_solutions").as_int());

  p.cartesian_max_velocity_scaling = get("cartesian_max_velocity_scaling").as_double();
  p.cartesian_max_acceleration_scaling = get("cartesian_max_acceleration_scaling").as_double();
  p.cartesian_step_size = get("cartesian_step_size").as_double();
  return p;
}

std::string join(const std::vector<double> & values)
{
  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    out << (i ? ", " : "") << std::to_string(values[i]);
  }
  return out.str();
}

}  // namespace pick_place
