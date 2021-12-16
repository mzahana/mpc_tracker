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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#include <mpc_tracker/navigator_node.h>

namespace navigator {
const double NavigatorNode::kCommandTimerFrequency = 5.0;
const double NavigatorNode::kWaypointAchievementDistance = 0.5;
const double NavigatorNode::kIntermediatePoseTolerance = 0.1;
const int NavigatorNode::kDimensions = 3;
const int NavigatorNode::kDerivativeToOptimize =
    mav_trajectory_generation::derivative_order::ACCELERATION;
const int NavigatorNode::kPolynomialCoefficients = 10;

NavigatorNode::NavigatorNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      got_odometry_(false)

{
  loadParameters();

  odometry_subscriber_ = nh_.subscribe(
      "odometry", 1, &NavigatorNode::odometryCallback, this);
  state_traj_subscriber_ = nh_.subscribe(
      "mpc_tracker/trajectory", 1, &NavigatorNode::stateTrajCallback, this);
  pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      mav_msgs::default_topics::COMMAND_POSE, 1);
  path_segments_publisher_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);

  // Visualization.
  path_points_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "waypoint_navigator_path_points_marker", 0);
  path_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "waypoint_navigator_path_marker", 0);
  polynomial_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "waypoint_navigator_polynomial_markers", 1, true);

  visualize_service_ = nh_.advertiseService(
      "visualize_path", &NavigatorNode::visualizePathCallback, this);
  takeoff_service_ = nh_.advertiseService(
      "takeoff", &NavigatorNode::takeoffCallback, this);
  land_service_ =
      nh_.advertiseService("land", &NavigatorNode::landCallback, this);
  abort_path_service_ = nh_.advertiseService(
      "abort_path", &NavigatorNode::abortPathCallback, this);
  waypoint_service_ = nh_.advertiseService(
      "go_to_waypoint", &NavigatorNode::goToWaypointCallback, this);
  waypoints_service_ = nh_.advertiseService(
      "go_to_waypoints", &NavigatorNode::goToWaypointsCallback, this);
  pose_waypoints_service_ = nh_.advertiseService(
      "go_to_pose_waypoints", &NavigatorNode::goToPoseWaypointsCallback, this);
  height_service_ = nh_.advertiseService(
      "go_to_height", &NavigatorNode::goToHeightCallback, this);

  // Wait until GPS reference parameters are initialized.
  while (!geodetic_converter_.isInitialised() && coordinate_type_ == "gps") {
    LOG_FIRST_N(INFO, 1) << "Waiting for GPS reference parameters...";

    double latitude;
    double longitude;
    double altitude;

    if (nh_private_.getParam("/gps_ref_latitude", latitude) &&
        nh_private_.getParam("/gps_ref_longitude", longitude) &&
        nh_private_.getParam("/gps_ref_altitude", altitude)) {
      geodetic_converter_.initialiseReference(latitude, longitude, altitude);
    } else {
      LOG(INFO) << "GPS reference not ready yet, use set_gps_reference_node to "
                   "set it.";
      ros::Duration(0.5).sleep();
    }
  }

  LOG(INFO)
      << "Waypoint navigator ready. Publish to mpc_tracker/trajectory topic to get going.";
}

void NavigatorNode::loadParameters() {
  CHECK(
      nh_private_.getParam("coordinate_type", coordinate_type_) &&
      nh_private_.getParam("path_mode", path_mode_) &&
      nh_private_.getParam("heading_mode", heading_mode_) &&
      nh_private_.getParam("reference_speed", reference_speed_) &&
      nh_private_.getParam("reference_acceleration", reference_acceleration_) &&
      nh_private_.getParam("takeoff_height", takeoff_height_) &&
      nh_private_.getParam("landing_height", landing_height_) &&
      nh_private_.getParam("mav_name", mav_name_) &&
      nh_private_.getParam("frame_id", frame_id_) &&
      nh_private_.getParam("intermediate_poses", intermediate_poses_))
      << "Error loading parameters!";

  if (coordinate_type_ == "gps" || coordinate_type_ == "enu") {
  } else {
    LOG(FATAL) << ("Unknown coordinate type - please enter 'gps' or 'enu'.");
  }

  if (path_mode_ == "poses" || path_mode_ == "polynomial") {
  } else {
    LOG(FATAL) << "Unknown path type - please enter 'poses', or 'trajectory'.";
  }

  if (heading_mode_ == "auto" || heading_mode_ == "manual" ||
      heading_mode_ == "zero") {
  } else {
    LOG(FATAL) << "Unknown heading alignment mode - please enter 'auto', "
                  "'manual', or 'zero'.";
  }

  if (intermediate_poses_) {
    CHECK(nh_private_.getParam("intermediate_pose_separation",
                               intermediate_pose_separation_))
        << "Cannot set intermediate poses without an intermediate pose "
           "separation.";
  }
}


void NavigatorNode::addIntermediateWaypoints() {
  for (size_t i = 1; i < coarse_waypoints_.size(); ++i) {
    mav_msgs::EigenTrajectoryPoint wpa = coarse_waypoints_[i - 1];
    mav_msgs::EigenTrajectoryPoint wpb = coarse_waypoints_[i];
    double dist = (wpa.position_W - wpb.position_W).norm();

    // Minimum tolerance between points set to avoid subsequent numerical errors
    // in trajectory optimization.
    while (dist > intermediate_pose_separation_ &&
           dist > kIntermediatePoseTolerance) {
      mav_msgs::EigenTrajectoryPoint iwp;
      iwp.position_W.x() = wpa.position_W.x() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.x() - wpa.position_W.x());
      iwp.position_W.y() = wpa.position_W.y() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.y() - wpa.position_W.y());
      iwp.position_W.z() = wpa.position_W.z() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.z() - wpa.position_W.z());
      iwp.orientation_W_B = wpb.orientation_W_B;
      coarse_waypoints_.insert(coarse_waypoints_.begin() + i, iwp);
      wpa = iwp;
      dist = (wpa.position_W - wpb.position_W).norm();
      i++;
    }
  }
}

void NavigatorNode::addCurrentOdometryWaypoint() {
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W = odometry_.position_W;
  vwp.orientation_W_B = odometry_.orientation_W_B;
  coarse_waypoints_.push_back(vwp);
}

void NavigatorNode::createTrajectory() {
  polynomial_vertices_.clear();
  polynomial_trajectory_.clear();
  yaw_vertices_.clear();
  yaw_trajectory_.clear();
  deletePolynomialMarkers();

  // Create a list of vertices.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    mav_trajectory_generation::Vertex vertex(kDimensions);
    mav_trajectory_generation::Vertex yaw(1);

    // Position.
    if (i == 0 || i == coarse_waypoints_.size() - 1) {
      vertex.makeStartOrEnd(coarse_waypoints_[i].position_W,
                            mav_trajectory_generation::derivative_order::SNAP);
    } else {
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          coarse_waypoints_[i].position_W);
    }
    // Yaw.
    if (i != 0) {
      // Check whether to rotate clockwise or counter-clockwise in yaw.
      double yaw_mod = fmod(
          coarse_waypoints_[i].getYaw() - coarse_waypoints_[i - 1].getYaw(),
          2 * M_PI);
      if (yaw_mod < -M_PI) {
        yaw_mod += 2 * M_PI;
      } else if (yaw_mod > M_PI) {
        yaw_mod -= 2 * M_PI;
      }
      coarse_waypoints_[i].setFromYaw(coarse_waypoints_[i - 1].getYaw() +
                                      yaw_mod);
    }
    yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      coarse_waypoints_[i].getYaw());

    polynomial_vertices_.push_back(vertex);
    yaw_vertices_.push_back(yaw);
  }

  // Optimize the polynomial trajectory.
  // Position.
  std::vector<double> segment_times;
  segment_times =
      estimateSegmentTimes(polynomial_vertices_, reference_speed_,
                           reference_acceleration_);

  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      opt(kDimensions);
  opt.setupFromVertices(polynomial_vertices_, segment_times,
                        kDerivativeToOptimize);
  opt.solveLinear();
  opt.getTrajectory(&polynomial_trajectory_);
  // Yaw.
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      yaw_opt(1);
  yaw_opt.setupFromVertices(yaw_vertices_, segment_times,
                            kDerivativeToOptimize);
  yaw_opt.solveLinear();
  yaw_opt.getTrajectory(&yaw_trajectory_);
}

void NavigatorNode::createTrajectoryWithConstrainst() {
  polynomial_vertices_.clear();
  polynomial_trajectory_.clear();
  yaw_vertices_.clear();
  yaw_trajectory_.clear();
  deletePolynomialMarkers();

  // Create a list of vertices.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    mav_trajectory_generation::Vertex vertex(kDimensions);
    mav_trajectory_generation::Vertex yaw(1);

    // Position.
    if (i == 0 || i == coarse_waypoints_.size() - 1) {
      vertex.makeStartOrEnd(coarse_waypoints_[i].position_W,
                            mav_trajectory_generation::derivative_order::SNAP);
    } else {
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          coarse_waypoints_[i].position_W);
      // vertex.addConstraint(
      //     mav_trajectory_generation::derivative_order::VELOCITY,
      //     coarse_waypoints_[i].velocity_W);
      // vertex.addConstraint(
      //     mav_trajectory_generation::derivative_order::ACCELERATION,
      //     coarse_waypoints_[i].acceleration_W);
    }
    // Yaw.
    if (i != 0) {
      // Check whether to rotate clockwise or counter-clockwise in yaw.
      double yaw_mod = fmod(
          coarse_waypoints_[i].getYaw() - coarse_waypoints_[i - 1].getYaw(),
          2 * M_PI);
      if (yaw_mod < -M_PI) {
        yaw_mod += 2 * M_PI;
      } else if (yaw_mod > M_PI) {
        yaw_mod -= 2 * M_PI;
      }
      coarse_waypoints_[i].setFromYaw(coarse_waypoints_[i - 1].getYaw() +
                                      yaw_mod);
    }
    yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      coarse_waypoints_[i].getYaw());

    polynomial_vertices_.push_back(vertex);
    yaw_vertices_.push_back(yaw);
  }

  // Optimize the polynomial trajectory.
  // Position.
  std::vector<double> segment_times;
  segment_times =
      estimateSegmentTimes(polynomial_vertices_, reference_speed_,
                           reference_acceleration_);

  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      opt(kDimensions);
  opt.setupFromVertices(polynomial_vertices_, segment_times,
                        kDerivativeToOptimize);
  opt.solveLinear();
  opt.getTrajectory(&polynomial_trajectory_);
  // Yaw.
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      yaw_opt(1);
  yaw_opt.setupFromVertices(yaw_vertices_, segment_times,
                            kDerivativeToOptimize);
  yaw_opt.solveLinear();
  yaw_opt.getTrajectory(&yaw_trajectory_);
}

void NavigatorNode::publishCommands() {
  if (path_mode_ == "poses") {
    command_timer_ =
        nh_.createTimer(ros::Duration(1.0 / kCommandTimerFrequency),
                        &NavigatorNode::poseTimerCallback, this);
  } else if (path_mode_ == "polynomial") {
    createTrajectory();
    // Publish the trajectory directly to the trajectory sampler.
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::Trajectory traj_with_yaw;
    polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
                                                              &traj_with_yaw);
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
        traj_with_yaw, &msg);
    path_segments_publisher_.publish(msg);
  }
}

void NavigatorNode::deletePolynomialMarkers() {
  for (size_t i = 0; i < markers_.markers.size(); ++i) {
    markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  polynomial_publisher_.publish(markers_);
}


bool NavigatorNode::goToWaypointCallback(
    mpc_tracker::GoToWaypoint::Request& request,
    mpc_tracker::GoToWaypoint::Response& response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_.stop();

  addCurrentOdometryWaypoint();

  // Add the new waypoint.
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W.x() = request.point.x;
  vwp.position_W.y() = request.point.y;
  vwp.position_W.z() = request.point.z;
  if (heading_mode_ == "zero") {
    vwp.setFromYaw(0.0);
  } else if (sqrt(pow(request.point.y - odometry_.position_W.y(), 2) +
                  pow(request.point.x - odometry_.position_W.x(), 2)) < 0.05) {
    vwp.orientation_W_B = odometry_.orientation_W_B;
  } else {
    vwp.setFromYaw(atan2(request.point.y - odometry_.position_W.y(),
                        request.point.x - odometry_.position_W.x()));
  }
  coarse_waypoints_.push_back(vwp);

  // Limit the maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  publishCommands();
  LOG(INFO) << "Going to a new waypoint...";
  return true;
}

bool NavigatorNode::goToWaypointsCallback(
    mpc_tracker::GoToWaypoints::Request& request,
    mpc_tracker::GoToWaypoints::Response& response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_.stop();

  addCurrentOdometryWaypoint();

  // Add points to a new path.
  std::vector<geometry_msgs::Point> points = request.points;
  mav_msgs::EigenTrajectoryPoint vwp;
  for (size_t i = 0; i < points.size(); ++i) {
    vwp.position_W.x() = points[i].x;
    vwp.position_W.y() = points[i].y;
    vwp.position_W.z() = points[i].z;
    coarse_waypoints_.push_back(vwp);
  }

  // Add heading to path.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    if (heading_mode_ == "auto") {
      if (i == 0) {
        continue;
      }
      // Compute heading in direction towards next point.
      coarse_waypoints_[i].setFromYaw(
          atan2(coarse_waypoints_[i].position_W.y() -
                    coarse_waypoints_[i - 1].position_W.y(),
                coarse_waypoints_[i].position_W.x() -
                    coarse_waypoints_[i - 1].position_W.x()));
    }
    // For both 'manual' and 'zero' heading modes, set zero heading.
    else {
      coarse_waypoints_[i].setFromYaw(0.0);
    }
  }

  // Display the path markers in rviz.
  visualization_timer_ =
      nh_.createTimer(ros::Duration(0.1),
                      &NavigatorNode::visualizationTimerCallback, this);
  publishCommands();
  return true;
}

bool NavigatorNode::goToPoseWaypointsCallback(
    mpc_tracker::GoToPoseWaypoints::Request& request,
    mpc_tracker::GoToPoseWaypoints::Response& response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_.stop();

  // Add points to a new path.
  std::vector<geometry_msgs::Pose> waypoints = request.waypoints;
  mav_msgs::EigenTrajectoryPoint vwp;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    vwp.position_W.x() = waypoints[i].position.x;
    vwp.position_W.y() = waypoints[i].position.y;
    vwp.position_W.z() = waypoints[i].position.z;

    vwp.orientation_W_B.x() = waypoints[i].orientation.x;
    vwp.orientation_W_B.y() = waypoints[i].orientation.y;
    vwp.orientation_W_B.z() = waypoints[i].orientation.z;
    vwp.orientation_W_B.w() = waypoints[i].orientation.w;

    if (i==0)
    {
      const double dist_to_end =
        (vwp.position_W - odometry_.position_W).norm();
          
      if (dist_to_end > kWaypointAchievementDistance) {
        LOG(INFO) << "Extra waypoint added because current pose is too far (" << dist_to_end << "m) from the first waypoint.";
        addCurrentOdometryWaypoint();
      }
    }

    coarse_waypoints_.push_back(vwp);
  }

  if(coarse_waypoints_.size() > 1)
  {
    LOG(INFO) << coarse_waypoints_.size()<<" waypoints received.";
    // Display the path markers in rviz.
    visualization_timer_ =
      nh_.createTimer(ros::Duration(0.1),
                      &NavigatorNode::visualizationTimerCallback, this);
    publishCommands();
  }else{
    LOG(INFO) << " Nothing to do because the destination is too close.";
    return false;
  }

  return true;
}

bool NavigatorNode::goToHeightCallback(
    mpc_tracker::GoToHeight::Request& request,
    mpc_tracker::GoToHeight::Response& response) {
  mpc_tracker::GoToWaypoint::Request vwp;
  vwp.point.x = odometry_.position_W.x();
  vwp.point.y = odometry_.position_W.y();
  vwp.point.z = request.height.data;
  mpc_tracker::GoToWaypoint::Response empty_response;
  return goToWaypointCallback(vwp, empty_response);
}

bool NavigatorNode::landCallback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {
  mpc_tracker::GoToWaypoint::Request vwp;
  vwp.point.x = odometry_.position_W.x();
  vwp.point.y = odometry_.position_W.y();
  vwp.point.z = landing_height_;
  mpc_tracker::GoToWaypoint::Response empty_response;
  return goToWaypointCallback(vwp, empty_response);
}

bool NavigatorNode::takeoffCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  mpc_tracker::GoToWaypoint::Request vwp;
  vwp.point.x = odometry_.position_W.x();
  vwp.point.y = odometry_.position_W.y();
  vwp.point.z = odometry_.position_W.z() + takeoff_height_;
  mpc_tracker::GoToWaypoint::Response empty_response;
  return goToWaypointCallback(vwp, empty_response);
}

bool NavigatorNode::abortPathCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  coarse_waypoints_.clear();
  polynomial_trajectory_.clear();
  polynomial_vertices_.clear();
  yaw_trajectory_.clear();
  yaw_vertices_.clear();
  visualization_timer_.stop();

  // Stop sending commands to the controller.
  if (path_mode_ == "polynomial") {
    std_srvs::Empty::Request empty_request;
    std_srvs::Empty::Response empty_response;
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty::Request>(
        "/" + mav_name_ + "/stop_sampling");
    client.call(empty_request, empty_response);
  } else {
    command_timer_.stop();
  }
  LOG(INFO) << "Aborting path execution...";
  return true;
}

bool NavigatorNode::visualizePathCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  CHECK(got_odometry_) << "No odometry received yet, can't visualize the path.";
  // CHECK(loadPathFromFile()) << "Path could not be loaded!";

  if (path_mode_ == "polynomial") {
    createTrajectory();
  }

  visualization_timer_ =
      nh_.createTimer(ros::Duration(0.1),
                      &NavigatorNode::visualizationTimerCallback, this);
  return true;
}

void NavigatorNode::poseTimerCallback(const ros::TimerEvent&) {
  // Check for leg completion based on distance.
  // If current leg has been completed, go to the next one.
  const double dist_to_end =
      (coarse_waypoints_[current_leg_].position_W - odometry_.position_W)
          .norm();

  if (current_leg_ != coarse_waypoints_.size() - 1 &&
      dist_to_end < kWaypointAchievementDistance) {
    if (current_leg_ == 0) {
      LOG(INFO) << "Going to first waypoint... ";
    } else {
      LOG(INFO) << "Leg " << current_leg_ << " of "
                << coarse_waypoints_.size() - 1 << " completed!";
    }
    current_leg_++;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.seq = timer_counter_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = coarse_waypoints_[current_leg_].position_W.x();
  pose.pose.position.y = coarse_waypoints_[current_leg_].position_W.y();
  pose.pose.position.z = coarse_waypoints_[current_leg_].position_W.z();
  tf::Quaternion orientation = tf::createQuaternionFromRPY(
      0.0, 0.0, coarse_waypoints_[current_leg_].getYaw());
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();

  pose_publisher_.publish(pose);
  timer_counter_++;
}

void NavigatorNode::visualizationTimerCallback(const ros::TimerEvent&) {
  // Fill out markers.
  visualization_msgs::Marker path_points_marker;
  path_points_marker.header.frame_id = frame_id_;
  path_points_marker.header.stamp = ros::Time();
  path_points_marker.ns = "waypoints";
  path_points_marker.id = 0;
  path_points_marker.type = visualization_msgs::Marker::CUBE_LIST;
  path_points_marker.action = visualization_msgs::Marker::ADD;
  path_points_marker.pose.position.x = 0.0;
  path_points_marker.pose.position.y = 0.0;
  path_points_marker.pose.position.z = 0.0;
  path_points_marker.pose.orientation.x = 0.0;
  path_points_marker.pose.orientation.y = 0.0;
  path_points_marker.pose.orientation.z = 0.0;
  path_points_marker.pose.orientation.w = 1.0;
  path_points_marker.scale.x = 0.1;
  path_points_marker.scale.y = 0.1;
  path_points_marker.scale.z = 0.1;
  path_points_marker.color.a = 1.0;
  path_points_marker.color.r = 1.0;
  path_points_marker.color.g = 0.0;
  path_points_marker.color.b = 0.0;

  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = frame_id_;
  path_marker.header.stamp = ros::Time();
  path_marker.ns = "path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.position.x = 0.0;
  path_marker.pose.position.y = 0.0;
  path_marker.pose.position.z = 0.0;
  path_marker.pose.orientation.x = 0.0;
  path_marker.pose.orientation.y = 0.0;
  path_marker.pose.orientation.z = 0.0;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.02;
  path_marker.color.a = 1.0;
  path_marker.color.r = 1.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 1.0;

  path_points_marker.points.clear();
  path_marker.points.clear();
  for (size_t i = 0; i < coarse_waypoints_.size(); ++i) {
    if (coarse_waypoints_.empty()) {
      continue;
    }
    geometry_msgs::Point pt;
    pt.x = coarse_waypoints_[i].position_W.x();
    pt.y = coarse_waypoints_[i].position_W.y();
    pt.z = coarse_waypoints_[i].position_W.z();
    path_points_marker.points.push_back(pt);
    path_marker.points.push_back(pt);
  }

  path_points_marker_publisher_.publish(path_points_marker);
  path_marker_publisher_.publish(path_marker);

  if (path_mode_ == "polynomial") {
    mav_trajectory_generation::Trajectory traj_with_yaw;
    polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
                                                              &traj_with_yaw);
    mav_trajectory_generation::drawMavTrajectory(traj_with_yaw, 1.0, frame_id_,
                                                 &markers_);
    polynomial_publisher_.publish(markers_);
  }
}

void NavigatorNode::odometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_message) {
  if (!got_odometry_) {
    got_odometry_ = true;
  }
  mav_msgs::eigenOdometryFromMsg(*odometry_message, &odometry_);
}

void NavigatorNode::stateTrajCallback(const custom_trajectory_msgs::StateTrajectoryConstPtr& msg)
{
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_.stop();

  //addCurrentOdometryWaypoint();

  reference_acceleration_ = msg->max_acceleration;
  reference_speed_ = msg->max_velocity;

  // Add points to a new path.
//   std::vector<geometry_msgs::Point> points = request.points;
  geometry_msgs::Point  vel_point;
  geometry_msgs::Point  accel_point;
  mav_msgs::EigenTrajectoryPoint vwp;
  for (size_t i = 0; i < msg->states.size(); ++i) {
    vwp.position_W.x() = msg->states[i].position.x;
    vwp.position_W.y() = msg->states[i].position.y;
    vwp.position_W.z() = msg->states[i].position.z;

    vwp.velocity_W.x() = msg->states[i].velocity.x;
    vwp.velocity_W.y() = msg->states[i].velocity.y;
    vwp.velocity_W.z() = msg->states[i].velocity.z;

    vwp.acceleration_W.x() = msg->states[i].acceleration.x;
    vwp.acceleration_W.y() = msg->states[i].acceleration.y;
    vwp.acceleration_W.z() = msg->states[i].acceleration.z;

    coarse_waypoints_.push_back(vwp);
  }

  // Add heading to path.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    if (heading_mode_ == "auto") {
      if (i == 0) {
        continue;
      }
      // Compute heading in direction towards next point.
      coarse_waypoints_[i].setFromYaw(
          atan2(coarse_waypoints_[i].position_W.y() -
                    coarse_waypoints_[i - 1].position_W.y(),
                coarse_waypoints_[i].position_W.x() -
                    coarse_waypoints_[i - 1].position_W.x()));
    }
    // For both 'manual' and 'zero' heading modes, set zero heading.
    else {
      coarse_waypoints_[i].setFromYaw(0.0);
    }
  }

  // Display the path markers in rviz.
  visualization_timer_ =
      nh_.createTimer(ros::Duration(0.1),
                      &NavigatorNode::visualizationTimerCallback, this);

  createTrajectoryWithConstrainst();

  // Publish the trajectory directly to the trajectory sampler.
  mav_planning_msgs::PolynomialTrajectory4D poly_msg;
  mav_trajectory_generation::Trajectory traj_with_yaw;
  polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
                                                            &traj_with_yaw);
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
      traj_with_yaw, &poly_msg);
  path_segments_publisher_.publish(poly_msg);
  return ;
}

}

int main(int argc, char** argv) {
  // Start the logging.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  // Initialize ROS, start node.
  ros::init(argc, argv, "NavigatorNode");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  navigator::NavigatorNode navigator_node(nh, nh_private);
  ros::spin();
  return 0;
}
