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
#ifndef MPC_TRACKER_H
#define MPC_TRACKER_H

#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h> // MAVROS setpoint msg
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "custom_trajectory_msgs/StateTrajectory.h"

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include<fstream>

#include <Eigen/Dense>

#include <nlopt.hpp>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

// For plotting
#include <plotty/matplotlibcpp.hpp>

class Commander
{
private:
  ros::NodeHandle nh_; /** ROS node handle */
  ros::NodeHandle nh_private_; /** ROS private node handle */
  ros::Subscriber mavstateSub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer armAndOffb_service_;
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer holdPos_service_;
  ros::Publisher setpoint_pub_; /** setpoints publisher */

  ros::Timer cmdloop_timer_; /** Setpoint publishing timer */
  double cmd_rate_; /** cmdloop_timer_ rate in seconds */

  mavros_msgs::State current_state_;
  
  ros::Time last_request_; /** Last time of MAV state request (current_state_) */
  ros::Time last_sp_update_req_; /** Last time setpoint_msg_ is updated externally */

  mavros_msgs::PositionTarget setpoint_msg_;
  geometry_msgs::PoseStamped dronePose_msg_; /** Current drone pose */

  double takeoff_alt_;

  bool hold_pos_; /** Flag for holding drone's position */

  /**
   * @brief Commander loop callback.
   */
  void cmdloopCallback(const ros::TimerEvent& event);

  /**
   * @brief ROS service for arming and setting OFFBOARD mode.
   */
  bool armAndOffbCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief ROS service for takeoff
   */
  bool takeoffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief ROS service for holding current position.
   */
  bool holdPosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief MAVROS state callback. Contains arming state and flight mode.
   */
  void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);

  /**
   * @brief Sets setpoint mode to position mode and holds current pose.
   */
  void holdPose(void);

public:
  /**
   * @brief Constructor
   * 
   * @param nh ROS node handle
   * @param nh_private Private ROS node handle
   */
  Commander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  
  /** Destructor */
  ~Commander();

  /**
   * @brief Setter for dronePose_msg_
   */
  void setDronePose(geometry_msgs::PoseStamped msg);

  /**
   * @brief Updates setpoint_msg_ and last_sp_update_req_
   */
  void updateSetpoint(mavros_msgs::PositionTarget msg);

  /**
   * @brief Published setpoint_msg_
   */
  void publishSetpoint(void);
};



//* MPCTracker class
/**
 * Implements Receding horizon controller for tracking a moving object.
 * Assumed model of the drone that is tracking the object is a 3rd order discrete LTI system capturing the translational dynamics.
 */
class MPCTracker
{
private:
  ros::NodeHandle       _nh;                    /** ROS node handle */
  ros::NodeHandle       _nh_private;            /** ROS private node handle */

  ros::Subscriber       _dronePose_sub;         /** REMOVE Drone's pose subscriber */
  ros::Subscriber       _droneOdom_sub;         /** Drone's odometry subscriber, to get position/velocity, "mavros/local_position/odom" */
  ros::Subscriber       _droneImu_sub;          /** Drone's IMU subscriber, to get acceleration, "mavros/imu/data" */
  ros::Subscriber       _referenceTraj_sub;     /** Subscriber to the target predicted trajectory. Referene trajectory of the MPC */
  ros::Subscriber       _testCase_sub;          /** Subscriber for running testCases() function */

  ros::Publisher        _desired_traj_pub;      /** Desired trajectory sent to the trajectory planner/sampler */
  ros::Publisher        _poseHistory_pub;       /** ROS Publisher for _posehistory_vector */
  ros::Publisher        _multiDofTraj_pub;          /** To publish first MPC control solution to the geometric controller */
  
  ros::ServiceServer    _engageCtrl_srv;        /** Engages/disengages MPC controller */

  double                _dt;                    /** Prediction time step in seconds */
  std::string           _reference_frame_id;    /** Name of the map (inertial) frame, where the drone localizes */
  
  bool                  _use_6dof_model;        /** Use 6DoF model instead of 9DoF */
  Eigen::MatrixXd       _current_drone_state;   /** Current drone state (position, velocity, acceleration) */
  Eigen::Matrix3d       _current_drone_accel;   /** Latest drone acceleration measurements. Will be added to _current_drone_state */
  bool                  _drone_state_received;  /** True if a drone's first measurment is received. Used for initialization*/
  ros::Time             _drone_state_last_t;    /** Last time stamp of _current_drone_state */
  ros::Time             _drone_state_current_t; /** Current time stamp of _current_drone_state */
  bool                  _target_traj_received;  /** Flag to indicate whether the first target trajectory is received */
  
  
  double                _alt_above_target;      /** Desired altitude above target. */
  
  Eigen::MatrixXd       _referenceTraj;         /** Target's predicted trajectory, over _mpcWindow. Received from the target predictor node*/
  Eigen::VectorXd       _ref_traj_px;           /** Reference trajectory, position, x component */
  Eigen::VectorXd       _ref_traj_py;           /** Reference trajectory, position, y component */
  Eigen::VectorXd       _ref_traj_pz;           /** Reference trajectory, position, z component */
  Eigen::VectorXd       _ref_traj_vx;           /** Reference trajectory, velocity, x component */
  Eigen::VectorXd       _ref_traj_vy;           /** Reference trajectory, velocity, y component */
  Eigen::VectorXd       _ref_traj_vz;           /** Reference trajectory, velocity, z component */
  Eigen::VectorXd       _ref_traj_ax;           /** Reference trajectory, acceleration, x component */
  Eigen::VectorXd       _ref_traj_ay;           /** Reference trajectory, acceleration, y component */
  Eigen::VectorXd       _ref_traj_az;           /** Reference trajectory, acceleration, z component */
  custom_trajectory_msgs::StateTrajectory _solution_traj_msg; /** ROS message for the optimal trajectory, position, velocity, acceleration, max velocity, max acceleration */
  ros::Time             _ref_traj_last_t;       /** Time stamp of the last reference trajectory */
  
  int                   _mpcWindow;             /** Number of prediction steps (N) */
  
  static const unsigned int      NUM_OF_STATES = 6;             /** Number of UAV states (position, velocity, acceleration) */
  static const unsigned int      NUM_OF_INPUTS = 3;      /** Number of control inputs of the UAV model. Jerk \in R^3 */
  Eigen::MatrixXd       _A;                     /** Discrete transition matrix */
  Eigen::MatrixXd       _B;                     /** Discrete input matrix */
  double                _state_weight;          /** State weight. */
  double                _input_weight;          /** Input weight. */
  double                _smooth_input_weight;   /** Weight/penality on input smoothing term */
  bool                  _enable_control_smoothing; /** Affects the Hessian structure */
  Eigen::MatrixXd       _Q;                     /** States wieght matrix of the quadratic MPC objective. x^T Q x. */
  Eigen::MatrixXd       _R;                     /** Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd       _gradient;              /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd       _hessian;               /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _hessian_sparse;   /** sparce version of the hessian */
  Eigen::MatrixXd       _Ac;                    /** Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _Ac_sparse;       /** Sparse version of Ac */ 
  Eigen::VectorXd       _xMin;                  /** Lower bounds on position, velocity, acceleration */
  Eigen::VectorXd       _xMax;                  /** Upper bounds on position, velocity, acceleration*/
  Eigen::VectorXd       _uMin;                  /** Lower bounds on inputs (jerk) */
  Eigen::VectorXd       _uMax;                  /** Upper bounds on inputs (jerk) */
  Eigen::VectorXd       _lowerBounds;            /** Lower bounds vector of the QP problem */
  Eigen::VectorXd       _upperBounds;           /** Upper bounds vector of the QP problem */
  Eigen::Vector3d       _maxVel;                /** Maximum drone's velocity m/s*/
  Eigen::Vector3d       _maxAccel;              /** Maximum drone's acceleration m/s/s */
  Eigen::Vector3d       _maxJerk;               /** Maximum drone's jerk m/s/s/s */

  Eigen::Vector3d       _mpc_ctrl_sol;          /** MPC control solution */
  Eigen::VectorXd       _optimal_state_traj;    /** Entire state trajectory part of the MPC solution */
  Eigen::VectorXd       _optimal_traj_px;       /** Optimal x-position trajectory */
  Eigen::VectorXd       _optimal_traj_py;       /** Optimal y-position trajectory */
  Eigen::VectorXd       _optimal_traj_pz;       /** Optimal z-position trajectory */
  Eigen::VectorXd       _optimal_traj_vx;       /** Optimal x-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_vy;       /** Optimal y-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_vz;       /** Optimal z-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_ax;       /** Optimal x-acceleration trajectory */
  Eigen::VectorXd       _optimal_traj_ay;       /** Optimal y-acceleration trajectory */
  Eigen::VectorXd       _optimal_traj_az;       /** Optimal z-acceleration trajectory */
  Eigen::VectorXd       _optimal_control_traj;  /** Entire control inputs trajectory part of the MPC solution */
  Eigen::VectorXd       _optimal_traj_ux;       /** Optimal control input trajectory in x direction */
  Eigen::VectorXd       _optimal_traj_uy;       /** Optimal control input trajectory in y direction */
  Eigen::VectorXd       _optimal_traj_uz;       /** Optimal control input trajectory in z direction */

  double                _minAltitude;           /** Minimum altitude to commanded by the MPC */

  bool                  _debug;                 /** Enable printing debug messages */
  bool                  _run_test_cases;        /** If true testCase() is executed, no ROS subscribers */
  bool                  _is_MPC_initialized;    /** True if MPC problem is initialized */
  bool                  _engage_controller;     /** **REMOVE** Flag for whether to publish MPC controls as setpoints */
  bool                  _is_mpc_engaged;        /** Flag for whether to publish MPC controls as setpoints */

  bool                  _pub_pose_path;         /** Whether to publish MPC predicted path for visualization in RViz */
  std::vector<geometry_msgs::PoseStamped> _posehistory_vector; /** Holds the predicted positions of the MPC, for visualization */

  Commander             *_commander;            /** SHOULD BE IN A SEPARATE NODE - OFFBOARD commander */

  OsqpEigen::Solver     _qpSolver;              /** Object of the quadratic program solver */

  bool                  _save_mpc_data;         /** Whether to save MPC data to _outputCSVFile */
  std::string           _outputCSVFile;         /** Full path to a CSV output file where MPC is stored */

  bool                  _plot;                  /** Plots solutions, reference trajectory. ONLY if _run_test_cases==True */

  /**
  * @brief Drone's odometry ROS Callback. Updates current_drone_state_ with position and velocity only
  * 
  * @param msg Holds Odometry measurement
  * 
  */
  void droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
  * @brief Drone's IMU ROS Callback. Updates current_drone_accel_
  * 
  * @param msg Holds sensor_msgs::Imu measurement
  * 
  */
  void droneImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
  * @brief Callback of the MPC reference trajectory, which is expected to be published by traj_predictor node.
  * Updates _referenceTraj
  * @param msg mpc_tracker::StateTrajectory
  */
  void refTrajCallback(const custom_trajectory_msgs::StateTrajectory::ConstPtr& msg);

  /**
  * @brief A callback that allows testCses() peroidically by publishing to a topic
  */
  void testCasesCallback(const std_msgs::Empty::ConstPtr& msg);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x.
   * Uses state_weight_ and updates Q_
   */
  void setQ(void);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x, fror 6DoF model.
   * Uses state_weight_ and updates Q_
   */
  void setQ6DoF(void);

  /**
   * @brief Sets the inputs weight matrix, R in u^T * R * u.
   * Uses input_weight_ and updates R_
   */
  void setR(void);

  /**
   * @brief Sets the transition matix for 3D discrete-time  integrator.
   * Result is a function of _dt and it's saved in _A
   */
  void setTransitionMatrix(void);

  /**
   * @brief Sets the transition matix for discrete time 6DoF model
   * Result is a function of _dt and it's saved in _A
   */
  void setTransitionMatrix6DoF(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time integrator in 3D.
   * Result is a function of dt_ and saved in _B
   */
  void setInputMatrix(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time discrete time 6DoF model.
   * Result is a function of dt_ and saved in _B
   */
  void setInputMatrix6DoF(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax
   * Uses _maxVel, _maxAccel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  void setStateBounds(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax, for 6Dof model.
   * Uses _maxVel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  void setStateBounds6DoF(void);

  /**
   * @brief Defines the control limits, _uMin, _uMax
   * Uses _maxJerk
   * Updates _uMin, _uMax
   */
  void setControlBounds(void);

  /**
   * @brief Computes the Hessain matrix of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * The Hessian matrix of quadratic function x^T*P*X is P
   * Result is saved in _hessian
   */
  void castMPCToQPHessian(void);

  /**
   * @brief Computes the gradient vector (linear term q in q^T*x) of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * Result is saved in _gradient
   */
  void castMPCToQPGradient(void);

  /**
   * @brief Updates the gradient vector, _gradient, using _referenceTraj
   */
  void updateQPGradientVector(void);

  /**
   * @brief Computes the constraint matrix required by the QP solver
   */
  void castMPCToQPConstraintMatrix(void);

  /**
   * @brief Constructs the bounds vectors for the QP problem.
   * Uses _xMin, _xMax, _uMin, _uMax
   * Result is saved in _lowerBounds and _upperBounds vetors.
   */
  void castMPCToQPConstraintBounds(void);

  /**
   * @brief Updates _lowerBounds & _upperBounds vectors, using _current_drone_state
   */
  void updateQPConstraintsBounds(void);

  /**
   * @brief Initialize QP solver
   * 
   * @return True if initialization is successful. False, otherwise.
   */
  bool initQPSolver(void);

  /**
   * @brief Initialize MPC problem.
   * @return Bool True if initialization is successful
   */
  bool initMPCProblem(void);

  /**
  * @brief Updates _qpSolver bounds & gradient using _current_drone_state, and _referenceTraj
  */
  bool updateQP(void);

  /**
   * @brief This is the main loop, which executes MPC control loop.
   */
  bool mpcLoop(void);

  /**
  * @brief Extracts MPC solutions, contorl/state trajectories, from QP
  * Updates _state_traj_sol, _control_traj_sol
  */
  void extractSolution(void);

  /**
  * @brief Extracts MPC solutions, contorl/state trajectories, from QP, for 6DoF model.
  * Updates _state_traj_sol, _control_traj_sol
  */
  void extractSolution6Dof(void);

  /**
   * @brief Published setpoint message to MAVROS.
   */
  void publishSetpoint(void);

  bool engageMPCCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
  * @brief Prints information about the QP problem, e.g. number of optimization variables, size of the Hessian matrix, ... etc
  */
  void printProblemInfo(void);

  /**
  * @brief Appends a new pose msg to _posehistory_vector
  */
  void appendPoseHistory(geometry_msgs::PoseStamped pose_msg);

  /**
  * @brief Publishes _posehistory_vector to a ROS topic
  */
  void pubPoseHistory(void);

  /**
   * @brief Published first MPC control solution u[0] to the geometric controller
  */ 
  void pubMultiDofTraj(void);

  /**
  * @brief Runs a test case to test the entire MPC loop and the associated functions
  */
  void testCases(void);

  /**
  * @brief Saves MPC matrices to an CSV file
  *
  * @param fileName Full path to .csv file
  */
  void saveMPCDataToFile(void);

  /**
  * @brief Plots solutions of the MPC problem. Works ONLY if _run_test_cases==True. For realtime plotting use RViz
  */
  void plotSolutions(void);

public:

  /**
   * @brief Constructor
   * 
   * @param nh ROS node handle
   * @param nh_private Private ROS node handle
   */
  MPCTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  
  /** Destructor */
  ~MPCTracker();
};

#endif //MPC_TRACKER_H