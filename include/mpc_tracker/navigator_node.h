/*
MIT License

Copyright (c) 2021 SystemTrio Robotics
Copyright (c) 2021 Mohamed Abdelkader, mohamedashraf123@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef NAVIGATOR_NODE_H
#define NAVIGATOR_NODE_H

#include <glog/logging.h>
#include <gtest/gtest_prod.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Path.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <geodetic_utils/geodetic_conv.hpp>
#include "tf/tf.h"
#include "trajectory_msgs/StateTrajectory.h"
#include "mpc_tracker/ExecutePathFromFile.h"
#include "mpc_tracker/GoToHeight.h"
#include "mpc_tracker/GoToPoseWaypoints.h"
#include "mpc_tracker/GoToWaypoints.h"
#include "mpc_tracker/GoToWaypoint.h"

namespace navigator {
class NavigatorNode {
 public:
  NavigatorNode(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  NavigatorNode(const NavigatorNode&) = delete;
  NavigatorNode& operator=(const NavigatorNode&) = delete;
  ~NavigatorNode() = default;

 private:
  // Fetches parameters from the ROS server.
  void loadParameters();

  // Interpolates intermediate points between waypoints in a sequence.
  void addIntermediateWaypoints();

  // Adds current MAV position as a waypoint to the path.
  void addCurrentOdometryWaypoint();

  // Creates and optimizes a smooth polynomial trajectory from a waypoint list.
  void createTrajectory();

  // Creates and optimizes a smooth polynomial trajectory from a waypoint list, with velocity/acceleration constraints.
  void createTrajectoryWithConstrainst();

  // Starts sending execution commands to the controller.
  void publishCommands();

  // Deletes old polynomial trajectory markers.
  void deletePolynomialMarkers();

  
  // Goes to a custom (x,y,z) waypoint.
  bool goToWaypointCallback(
      mpc_tracker::GoToWaypoint::Request& request,
      mpc_tracker::GoToWaypoint::Response& response);
  // Goes to a custom sequence of(x,y,z) waypoints.
  // Note: Does not add intermediate poses.
  bool goToWaypointsCallback(
      mpc_tracker::GoToWaypoints::Request& request,
      mpc_tracker::GoToWaypoints::Response& response);

  // Goes to a custom sequence of Pose waypoints, but only yaw is used.
  // Note: Does not add intermediate poses.
  bool goToPoseWaypointsCallback(
      mpc_tracker::GoToPoseWaypoints::Request& request,
      mpc_tracker::GoToPoseWaypoints::Response& response);


  // Goes to a specific height with current x-y position.
  bool goToHeightCallback(mpc_tracker::GoToHeight::Request& request,
                          mpc_tracker::GoToHeight::Response& response);
  // Send a landing command.
  bool landCallback(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response);
  // Send a take-off command.
  bool takeoffCallback(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response);
  // Cancel mission and keep helicopter in current position
  bool abortPathCallback(std_srvs::Empty::Request& request,
                         std_srvs::Empty::Response& response);
  // Publish path rviz markers given most recent odometry measurement.
  bool visualizePathCallback(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response);
  // Publishes a single waypoint to go to if the path mode is 'poses' [5Hz].
  void poseTimerCallback(const ros::TimerEvent&);
  // Show path commands published in rviz
  void visualizationTimerCallback(const ros::TimerEvent&);

  void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_message);

  void stateTrajCallback(const trajectory_msgs::StateTrajectoryConstPtr& msg);

  static const double kCommandTimerFrequency;
  // Distance before a waypoint is considered reached [m].
  static const double kWaypointAchievementDistance;
  // Minimum distance between intermediate waypoints [m].
  static const double kIntermediatePoseTolerance;
  // Number of dimensions in the problem.
  static const int kDimensions;
  // Order of derivative to optimize polynomial trajectory for.
  static const int kDerivativeToOptimize;
  // Number of coefficients of polynomial trajectory.
  static const int kPolynomialCoefficients;

  // ROS comms.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pose_publisher_;
  ros::Publisher path_segments_publisher_;
  ros::Publisher path_points_marker_publisher_;
  ros::Publisher path_marker_publisher_;
  ros::Publisher polynomial_publisher_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber state_traj_subscriber_;

  ros::ServiceServer visualize_service_;
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer land_service_;
  ros::ServiceServer abort_path_service_;
  ros::ServiceServer waypoint_service_;
  ros::ServiceServer waypoints_service_;
  ros::ServiceServer pose_waypoints_service_;
  ros::ServiceServer height_service_;

  ros::Timer command_timer_;
  ros::Timer visualization_timer_;

  // Parameters.
  // GPS/ENU coordinates.
  std::string coordinate_type_;
  // Trajectory/poses command publisher.
  std::string path_mode_;
  // Heading alignment method.
  std::string heading_mode_;
  std::string mav_name_;
  std::string frame_id_;
  // Addition of intermediate command poses.
  bool intermediate_poses_;
  // Maximum distance between poses [m].
  double intermediate_pose_separation_;
  // Maximum speed (m/s).
  double reference_speed_;
  // Maximum acceleration (m/s^2).
  double reference_acceleration_;
  // Height for takeoff command [m].
  double takeoff_height_;
  // Height for landing command [m].
  double landing_height_;

  // Geodetic coordinate conversion (from lat/lon to Cartesian ENU).
  geodetic_converter::GeodeticConverter geodetic_converter_;

  bool got_odometry_;
  mav_msgs::EigenOdometry odometry_;

  // A list of waypoints to visit.
  // [x,y,z,heading]
  std::vector<mav_msgs::EigenTrajectoryPoint> coarse_waypoints_;

  // Polynomial trajectory markers.
  visualization_msgs::MarkerArray markers_;

  // Path execution state (for pose publishing).
  size_t current_leg_;

  // Path vertices and segments.
  mav_trajectory_generation::Trajectory polynomial_trajectory_;
  mav_trajectory_generation::Vertex::Vector polynomial_vertices_;
  mav_trajectory_generation::Trajectory yaw_trajectory_;
  mav_trajectory_generation::Vertex::Vector yaw_vertices_;

  // Callback number for command_timer_.
  unsigned int timer_counter_;
};
}  // namespace: navigator

#endif  // NAVIGATOR_NODE_H
