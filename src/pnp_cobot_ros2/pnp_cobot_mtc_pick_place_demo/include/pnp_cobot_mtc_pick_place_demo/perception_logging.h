/**
 * @file perception_logging.h
 * @brief Logging for the point cloud processing libraries.
 *
 * Step-by-step progress goes to DEBUG, so it is hidden unless requested:
 *   ros2 launch ... --ros-args --log-level get_planning_scene_server:=debug
 * The logger is a child of the get_planning_scene_server node's logger, so that one
 * setting controls both the node and these libraries.
 *
 * @author Souvik Roy <sroyy57@gmail.com>
 */

#ifndef PNP_COBOT_MTC_PICK_PLACE_DEMO__PERCEPTION_LOGGING_H_
#define PNP_COBOT_MTC_PICK_PLACE_DEMO__PERCEPTION_LOGGING_H_

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace perception_logging
{
inline rclcpp::Logger logger()
{
  return rclcpp::get_logger("get_planning_scene_server.perception");
}
}  // namespace perception_logging

// Progress details (DEBUG) and failures (ERROR); the argument can be a string or a stream chain
#define LOG_INFO(x) RCLCPP_DEBUG_STREAM(perception_logging::logger(), x)
#define LOG_ERROR(x) RCLCPP_ERROR_STREAM(perception_logging::logger(), x)

#endif  // PNP_COBOT_MTC_PICK_PLACE_DEMO__PERCEPTION_LOGGING_H_
