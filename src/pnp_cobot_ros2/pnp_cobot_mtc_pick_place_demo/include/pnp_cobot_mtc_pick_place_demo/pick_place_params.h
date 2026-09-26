/**
 * @file pick_place_params.h
 * @brief Parameters of the MTC pick and place task, and their declaration and loading.
 *
 * Every parameter is declared once here with its default and description. The values are
 * set in config/mtc_node_params.yaml.
 *
 * @author Addison Sears-Collins
 * Modified by: Souvik Roy <sroyy57@gmail.com> (2026) — split out of mtc_node.cpp
 */

#ifndef PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_PARAMS_H_
#define PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_PARAMS_H_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace pick_place
{

/**
 * @brief All parameters used to build and run the pick and place task.
 *
 * Poses are [x, y, z, roll, pitch, yaw]. Object dimensions are [height, radius] for a
 * cylinder and [x, y, z] for a box.
 */
struct PickPlaceParams
{
  // General
  bool execute;
  int max_solutions;
  std::vector<std::string> controller_names;

  // Robot configuration (names from the SRDF)
  std::string arm_group_name;
  std::string gripper_group_name;
  std::string gripper_frame;
  std::string gripper_open_pose;
  std::string gripper_close_pose;
  std::string arm_home_pose;
  std::string world_frame;

  // Target object (overwritten with what perception found)
  std::string object_name;
  std::string object_type;
  std::string object_reference_frame;
  std::vector<double> object_dimensions;
  std::vector<double> object_pose;

  // Grasp and place
  std::vector<double> grasp_frame_transform;
  std::vector<double> place_pose;
  double place_pose_z_offset_factor;

  // Cartesian motion distances
  double approach_object_min_dist;
  double approach_object_max_dist;
  double lift_object_min_dist;
  double lift_object_max_dist;
  double lower_object_min_dist;
  double lower_object_max_dist;
  double retreat_min_distance;
  double retreat_max_distance;

  // Cartesian motion directions (z component)
  double approach_object_direction_z;
  double lift_object_direction_z;
  double lower_object_direction_z;
  double retreat_direction_z;

  // Connect stage timeouts (seconds)
  double move_to_pick_timeout;
  double move_to_place_timeout;

  // Grasp and place pose generation
  double grasp_pose_angle_delta;
  int grasp_pose_max_ik_solutions;
  double grasp_pose_min_solution_distance;
  int place_pose_max_ik_solutions;

  // Cartesian planner
  double cartesian_max_velocity_scaling;
  double cartesian_max_acceleration_scaling;
  double cartesian_step_size;
};

/**
 * @brief Declare every pick and place parameter on the node, with defaults and descriptions.
 *
 * Parameters that already exist (for example from the YAML file) are left unchanged.
 */
void declareParameters(rclcpp::Node & node);

/**
 * @brief Read the current values of all pick and place parameters from the node.
 */
PickPlaceParams loadParameters(const rclcpp::Node & node);

/**
 * @brief Format numbers as "a, b, c" for log messages.
 */
std::string join(const std::vector<double> & values);

}  // namespace pick_place

#endif  // PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_PARAMS_H_
