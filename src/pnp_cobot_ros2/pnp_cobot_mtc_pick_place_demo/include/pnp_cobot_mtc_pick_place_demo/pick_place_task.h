/**
 * @file pick_place_task.h
 * @brief Construction of the MoveIt Task Constructor (MTC) pick and place task.
 *
 * The task, in order:
 *   current state -> open gripper -> move to pick ->
 *   pick object  [approach, grasp pose + IK, allow collisions, close gripper, attach, lift] ->
 *   move to place ->
 *   place object [lower, place pose + IK, open gripper, forbid collision, detach, retreat] ->
 *   move home
 *
 * @author Addison Sears-Collins
 * Modified by: Souvik Roy <sroyy57@gmail.com> (2026) — split out of mtc_node.cpp
 */

#ifndef PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_TASK_H_
#define PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_TASK_H_

#include <string>

#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

#include "pnp_cobot_mtc_pick_place_demo/pick_place_params.h"

namespace pick_place
{

/**
 * @brief Build the pick and place task for the object named in params.
 *
 * @param node Node used to load the robot model and create the OMPL planner
 * @param params Task parameters, with the object fields set to the perceived target
 * @param support_surface_id Collision object the target stands on (collisions with it are
 *        allowed while grasping and lifting)
 * @return The task, ready for init() and plan()
 */
moveit::task_constructor::Task buildPickPlaceTask(
  const rclcpp::Node::SharedPtr & node,
  const PickPlaceParams & params,
  const std::string & support_surface_id);

}  // namespace pick_place

#endif  // PNP_COBOT_MTC_PICK_PLACE_DEMO__PICK_PLACE_TASK_H_
