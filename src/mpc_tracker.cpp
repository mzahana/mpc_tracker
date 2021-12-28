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

/** Brief Algorithm Description
Detailed description is available in [include reference]

* target_detector --> target_state_estimator (E/KF) --> traj_predictor --> mpc_tracker (this code) --> traj_planner --> traj_smapler --> so3_controller --> PX4

* refTrajCallback() is the main loop that executes the MPC steps and publishes the desired trajectory to the trajectory planner/sampler
* refTrajCallback() is called whenever a reference trajectory is published by the traj_predictor node
*/

#include "mpc_tracker/mpc_tracker.h"

MPCTracker::MPCTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
_nh(nh),
_nh_private(nh_private),
_dt(0.05),
_mpcWindow(20),
_state_weight(1.0),
_input_weight(0.1),
_smooth_input_weight(10),
_alt_above_target(1.0),
_is_MPC_initialized(false),
_drone_state_received(false),
_target_traj_received(false),
_debug(true),
_engage_controller(false),
_enable_control_smoothing(true),
_ref_traj_last_t(ros::Time::now()),
_drone_state_current_t(ros::Time::now()),
_drone_state_last_t(ros::Time::now()),
_pub_pose_path(false),
_plot(false),
_use_6dof_model(true),
_reference_frame_id("map"),
_minAltitude(1.0)
{
   _nh_private.param("debug", _debug, true);
   _nh_private.param("pub_pose_path", _pub_pose_path, false);
   _nh_private.param("run_test_cases", _run_test_cases, false);
   _nh_private.param("mpc_window", _mpcWindow, 1);
   _nh_private.param("state_weight", _state_weight, 1.0);
   _nh_private.param("input_weight", _input_weight, 0.1);
   _nh_private.param("smooth_input_weight", _smooth_input_weight, 0.1);
   _nh_private.param("enable_control_smoothing", _enable_control_smoothing, true);
   _nh_private.param("dt", _dt, 0.05);
   _nh_private.param("plot", _plot, false);
   _nh_private.param("alt_above_target", _alt_above_target, 2.0);
   _nh_private.param("save_mpc_data", _save_mpc_data, false);
   _nh_private.param("use_6dof_model", _use_6dof_model, true);
   _nh_private.param<std::string>("reference_frame_id", _reference_frame_id, "map");
   _nh_private.param<std::string>("output_csv_file", _outputCSVFile, "mpc_data.csv");
   _nh_private.param("minimum_altitude", _minAltitude, 1.0);

   //  Sanity check
   if (_use_6dof_model)
   {
      if (NUM_OF_STATES != 6)
      {
         ROS_ERROR("[MPCTracker] NUM_OF_STATES should be equal to 6 for 6-state model. Exiting...");
         return;
      }
   }
   else
   {
      if (NUM_OF_STATES != 9)
      {
         ROS_ERROR("[MPCTracker] NUM_OF_STATES should be equal to 9 for 9-state model. Exiting...");
         return;
      }
   }

   XmlRpc::XmlRpcValue maxVelConfig;
   if (_nh_private.hasParam("max_velocity"))
   {
      _nh_private.getParam("max_velocity", maxVelConfig);
      ROS_ASSERT(maxVelConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);  

      _maxVel.setZero();
      _maxVel(0) = maxVelConfig[0]; _maxVel(1) = maxVelConfig[1]; _maxVel(2) = maxVelConfig[2];


   }
   
   XmlRpc::XmlRpcValue maxAccConfig;
   if (_nh_private.hasParam("max_acceleration"))
   {
      _nh_private.getParam("max_acceleration", maxAccConfig);
      ROS_ASSERT(maxAccConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);  

      _maxAccel.setZero();
      _maxAccel(0) = maxAccConfig[0]; _maxAccel(1) = maxAccConfig[1]; _maxAccel(2) = maxAccConfig[2];


   }

   // max_jerk is currently not used
   XmlRpc::XmlRpcValue maxJerkConfig;
   if (_nh_private.hasParam("max_jerk"))
   {
      _nh_private.getParam("max_jerk", maxJerkConfig);
      ROS_ASSERT(maxJerkConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);  

      _maxJerk.setZero();
      _maxJerk(0) = maxJerkConfig[0]; _maxJerk(1) = maxJerkConfig[1]; _maxJerk(2) = maxJerkConfig[2];


   }

   // Subscribers
   if (!_run_test_cases)
   {
      _droneOdom_sub =  _nh.subscribe("mavros/local_position/odom", 1, &MPCTracker::droneOdomCallback, this, ros::TransportHints().tcpNoDelay());
      _droneImu_sub =  _nh.subscribe("mavros/imu/data", 1, &MPCTracker::droneImuCallback, this, ros::TransportHints().tcpNoDelay());
      _referenceTraj_sub = _nh.subscribe("traj_predictor/ref_traj",1, &MPCTracker::refTrajCallback, this, ros::TransportHints().tcpNoDelay());
   }
   else
   {
      _testCase_sub = _nh.subscribe("mpc_tracker/test_cases",1, &MPCTracker::testCasesCallback, this);
   }

   // Publishers
   _poseHistory_pub = _nh.advertise<nav_msgs::Path>("mpc_tracker/path", 10);
   _desired_traj_pub = _nh.advertise<custom_trajectory_msgs::StateTrajectory>("mpc_tracker/trajectory", 10);
   _multiDofTraj_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);

   // Services
   _engageCtrl_srv = _nh.advertiseService("mpc_commander/engage", &MPCTracker::engageMPCCallback, this);

   // commander_ = new Commander(nh, nh_private);

   if(!initMPCProblem())
   {
      ROS_ERROR("[MPCTracker] Could not initialize MPC problem");
      return;
   }

  //nlopt::opt *opt = new nlopt::opt(nlopt::LD_MMA, num_of_states_*mpcWindow + num_of_inputs_*mpcWindow);

  // Decide on the size of vectors in the initialization instead of doing it repeatidly in the callbacks
  // This is to avoid repeated memory (re)allocation, to save time.
  _referenceTraj.resize(NUM_OF_STATES*(_mpcWindow+1),1);

  printProblemInfo();

  if (_run_test_cases)
  {
     testCases();
  }

}

MPCTracker::~MPCTracker()
{
   /* Destructor */
}
void MPCTracker::printProblemInfo(void)
{
   auto opt_x_l = NUM_OF_STATES*(_mpcWindow+1)+NUM_OF_INPUTS*_mpcWindow; // length of optimization variable
   auto opt_Ac_l_row = 2*NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow;// number of constraints in matrix Ac

   auto opt_Ac_size = opt_Ac_l_row * opt_x_l; // Number of elements in Ac

   ROS_INFO("[MPC_TRACKER]: Number of states = %d",NUM_OF_STATES );
   ROS_INFO("[MPC_TRACKER]: Number of inputs = %d",NUM_OF_INPUTS );
   ROS_INFO("[MPC_TRACKER]: Number of MPC steps = %d",_mpcWindow );
   ROS_INFO("[MPC_TRACKER]: Length of MPC horizon = %f seconds",(double)_mpcWindow*_dt );
   ROS_INFO("[MPC_TRACKER]: Number of optimization variables = %d",opt_x_l );
   ROS_INFO("[MPC_TRACKER]: Number constraints = %d", opt_Ac_l_row );
   ROS_INFO("[MPC_TRACKER]: Number of elements in the constraints matrix Ac = %d",opt_Ac_size );
}

void MPCTracker::setTransitionMatrix(void)
{
   // states: [px, vx, ax, py, vy, ay, pz, vz, az]

   Eigen::MatrixXd A_dt;
   A_dt = Eigen::MatrixXd::Identity(3,3); // Construct the smaller A matrix

   A_dt(0,1) = _dt; //A_dt(0,2) = _dt*_dt/2.0;
   A_dt(1,2) = _dt;
   //A_dt(2,2)=0; // ref: Model Predictive Trajectory Tracking and Collision Avoidance for Reliable Outdoor Deployment of Unmanned Aerial Vehicles

   _A.resize(NUM_OF_STATES,NUM_OF_STATES);
   _A = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_STATES);
   
   _A.block(0,0,3, 3) = A_dt;
   _A.block(3,3,3, 3) = A_dt;
   _A.block(6,6,3, 3) = A_dt;

   if(_debug)
   {
      std::cout<< "Transition matrix A = " <<std::endl<< _A <<std::endl;
   }
}

void MPCTracker::setTransitionMatrix6DoF(void)
{
   // states: [px, vx, py, vy, pz, vz]

   Eigen::MatrixXd A_dt;
   A_dt = Eigen::MatrixXd::Identity(2,2); // Construct the smaller A matrix

   A_dt(0,1) = _dt;

   _A.resize(NUM_OF_STATES,NUM_OF_STATES);
   _A = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_STATES);
   
   _A.block(0,0, 2, 2) = A_dt;
   _A.block(2,2, 2, 2) = A_dt;
   _A.block(4,4, 2, 2) = A_dt;

   if(_debug)
   {
      std::cout<< "Transition matrix A = " <<std::endl<< _A <<std::endl;
   }
}

void MPCTracker::setInputMatrix(void)
{
   _B.resize(NUM_OF_STATES, NUM_OF_INPUTS);
   _B.setZero();

   Eigen::MatrixXd B_dt;
   B_dt = Eigen::MatrixXd::Zero(3,1); // Build the blocks of _B
   // B_dt(0,0) = _dt*_dt/2; // _dt*_dt*_dt/6.0; 
   // B_dt(1,0) = _dt; //_dt*_dt/2.0;
   B_dt(2,0) = _dt;

   // block(start_row_index, start_col_index, number_of_rows, number_of_cols)
   _B.block(0,0,3,1) = B_dt;
   _B.block(3,1,3,1) = B_dt;
   _B.block(6,2,3,1) = B_dt;

   if(_debug)
   {
      std::cout<<"Input matrix B = " << std::endl<< _B <<std::endl;
   }
}


void MPCTracker::setInputMatrix6DoF(void)
{
   _B.resize(NUM_OF_STATES, NUM_OF_INPUTS);
   _B.setZero();

   Eigen::MatrixXd B_dt;
   B_dt = Eigen::MatrixXd::Zero(2,1); // Build the blocks of _B
   B_dt(1,0) = _dt;

   _B.block(0,0, 2,1) = B_dt;
   _B.block(2,1, 2,1) = B_dt;
   _B.block(4,2, 2,1) = B_dt;

   if(_debug)
   {
      std::cout<<"Input matrix B = " << std::endl<< _B <<std::endl;
   }
}

void MPCTracker::setQ(void)
{
   
   _Q.resize(NUM_OF_STATES,NUM_OF_STATES); _Q.setZero();
   _Q(0,0) = _state_weight; // penality on position, x
   _Q(3,3) = _state_weight; // penality on position, y
   _Q(6,6) = _state_weight; // penality on position, z

   // // The following values are according to Martin Sask'a paper
   // _Q(1,2) = 800; // velocity-acceleration in x
   // _Q(2,1) = 800; // velocity-acceleration in x

   // _Q(4,5) = 800; // velocity-acceleration in y
   // _Q(5,4) = 800; // velocity-acceleration in y

   // _Q(7,8) = 800; // velocity-acceleration in z
   // _Q(8,7) = 800; // velocity-acceleration in z

   // _Q = _state_weight*Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_STATES);


   if(_debug)
   {
      std::cout<<"Q matrix = "<<std::endl<< _Q <<std::endl;
   }
}

void MPCTracker::setQ6DoF(void)
{
   
   _Q.resize(NUM_OF_STATES,NUM_OF_STATES); _Q.setZero();
   _Q(0,0) = _state_weight; // penality on position, x
   _Q(2,2) = _state_weight; // penality on position, y
   _Q(4,4) = _state_weight; // penality on position, z

   if(_debug)
   {
      std::cout<<"Q matrix = "<<std::endl<< _Q <<std::endl;
   }
}

void MPCTracker::setR(void)
{
   _R.resize(NUM_OF_INPUTS,NUM_OF_INPUTS);
   _R = _input_weight * Eigen::MatrixXd::Identity(NUM_OF_INPUTS, NUM_OF_INPUTS);

   if(_debug)
   {
      std::cout<<"R matrix: "<<std::endl<< _R <<std::endl;
   }
}

void MPCTracker::castMPCToQPHessian(void)
{
   int h_size = NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _hessian.resize(h_size, h_size);
   _hessian = Eigen::MatrixXd::Zero(h_size, h_size);

   // Add _Q to _hessian
   for (unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _hessian.block(NUM_OF_STATES*i, NUM_OF_STATES*i, NUM_OF_STATES, NUM_OF_STATES) = _Q;
   }

   // Add _R to _hessian
   unsigned int idx = (_mpcWindow+1)*NUM_OF_STATES; //initial index after adding _Q
   for (unsigned int i=0; i<_mpcWindow; i++)
   {
      _hessian.block(idx+i*NUM_OF_INPUTS, idx+i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = _R;
   }

   if (_enable_control_smoothing)
   {
      Eigen::MatrixXd s = _smooth_input_weight * Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
      Eigen::MatrixXd S = Eigen::MatrixXd::Identity(NUM_OF_INPUTS*(_mpcWindow-1), NUM_OF_INPUTS*(_mpcWindow-1));
      // Input difference matrix
      Eigen::MatrixXd U_diff = Eigen::MatrixXd::Zero(NUM_OF_INPUTS*(_mpcWindow-1), NUM_OF_INPUTS*_mpcWindow);
      for (int i=0; i<(_mpcWindow-1); i++)
      {
         S.block(i*NUM_OF_INPUTS, i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = s;
         U_diff.block(i*NUM_OF_INPUTS, i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = -1.0*Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
         U_diff.block(i*NUM_OF_INPUTS, (i+1)*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
      }

      Eigen::MatrixXd product = U_diff.transpose()*S*U_diff;
      auto N_x = NUM_OF_STATES*(_mpcWindow+1);
      auto N_u = NUM_OF_INPUTS*_mpcWindow;
      _hessian.block(N_x,N_x, N_u,N_u) = _hessian.block(N_x,N_x, N_u,N_u) + product;
   }


   if(_debug)
   {
      std::cout << "QP Hessian matrix P = " << std::endl << _hessian << std::endl;
   }
}

void MPCTracker::castMPCToQPGradient(void)
{
   int g_size = NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow;
   _gradient.resize(g_size);
   _gradient.setZero();

   // Populate the gradient vector
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _gradient.segment(i*NUM_OF_STATES,NUM_OF_STATES) = -1.0*_Q*_referenceTraj.block(i*NUM_OF_STATES,0,NUM_OF_STATES,1);
   }

   if(_debug)
   {
      std::cout<<"QP gradient vector q = "<<std::endl<<_gradient<<std::endl;
   }
}

void MPCTracker::updateQPGradientVector(void)
{
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _gradient.segment(i*NUM_OF_STATES,NUM_OF_STATES) = -1.0*_Q*_referenceTraj.block(i*NUM_OF_STATES,0,NUM_OF_STATES,1);
   }
   if (_debug)
   {
      std::cout << "[MPCTracker::updateQPGradientVector] Updated QP gradient = \n" << _gradient << "\n";
   }
}

// Ac matrix
void MPCTracker::castMPCToQPConstraintMatrix(void)
{
   // Initialize Ac
   int size_r = 2*NUM_OF_STATES * (_mpcWindow+1) + NUM_OF_INPUTS * _mpcWindow;
   int size_c = NUM_OF_STATES * (_mpcWindow+1) + NUM_OF_INPUTS * _mpcWindow;
   _Ac.resize(size_r, size_c);
   _Ac.setZero();
   //_Ac = Eigen::MatrixXd::Zero(size_r, size_c);

   // length of states/inputs over _mpcWindow
   auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS * _mpcWindow;

   _Ac.block(0, 0, N_x, N_x) = -1.0 * Eigen::MatrixXd::Identity(N_x,N_x);
   for (unsigned int i=1; i<_mpcWindow+1; i++)
   {
      // upper-left block
      _Ac.block(i*NUM_OF_STATES, (i-1)*NUM_OF_STATES, NUM_OF_STATES, NUM_OF_STATES) = _A;
      // upper-right block
      _Ac.block(i*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = _B;
      
      // input constraints: lower-right block
      //_Ac.block(2*N_x+(i-1)*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_INPUTS);
   }

   // Bounds: State block, middle-left block = Identity
   _Ac.block(N_x, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x,N_x);

   // Bounds: Inputs block, lower-right block = Identity
   _Ac.block(2*N_x, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u,N_u);

   if(_debug)
   {
      std::cout<<"Constraint matrix A_c = " <<std::endl<< _Ac <<std::endl;
   }
}

void MPCTracker::setStateBounds(void)
{
   _xMin.resize(NUM_OF_STATES);
   _xMin.setZero();
   _xMax.resize(NUM_OF_STATES);
   _xMax.setZero();

   // TODO Sanity checks on the values of maxVel, maxAccel

   // states order: [px, vx, ax, py, vy, ay, pz, vz, az]
   _xMin(0) = -1.0*OsqpEigen::INFTY;   _xMax(0) = OsqpEigen::INFTY; // px
   _xMin(1) = -1.0*_maxVel(0);         _xMax(1) = _maxVel(0); // vx
   _xMin(2) = -1.0*_maxAccel(0);       _xMax(2) = _maxAccel(0); // ax
   
   _xMin(3) = -1.0*OsqpEigen::INFTY;   _xMax(3) = OsqpEigen::INFTY; // py
   _xMin(4) = -1.0*_maxVel(1);         _xMax(4) = _maxVel(1); // vy
   _xMin(5) = -1.0*_maxAccel(1);       _xMax(5) = _maxAccel(1); // ay

   _xMin(6) = _minAltitude;            _xMax(6) = OsqpEigen::INFTY; // pz
   _xMin(7) = -1.0*_maxVel(2);         _xMax(7) = _maxVel(2); // vz
   _xMin(8) = -1.0*_maxAccel(2);       _xMax(8) = _maxAccel(2); // az
}

void MPCTracker::setStateBounds6DoF(void)
{
   _xMin.resize(NUM_OF_STATES);
   _xMin.setZero();
   _xMax.resize(NUM_OF_STATES);
   _xMax.setZero();

   // TODO Sanity checks on the values of maxVel, maxAccel

   // states order: [px, vx, py, vy, pz, vz]
   _xMin(0) = -1.0*OsqpEigen::INFTY;   _xMax(0) = OsqpEigen::INFTY; // px
   _xMin(1) = -1.0*_maxVel(0);         _xMax(1) = _maxVel(0); // vx
   
   _xMin(2) = -1.0*OsqpEigen::INFTY;   _xMax(2) = OsqpEigen::INFTY; // py
   _xMin(3) = -1.0*_maxVel(1);         _xMax(3) = _maxVel(1); // vy

   _xMin(4) = _minAltitude;            _xMax(4) = OsqpEigen::INFTY; // pz
   _xMin(5) = -1.0*_maxVel(2);         _xMax(5) = _maxVel(2); // vz
}

void MPCTracker::setControlBounds(void)
{
   // _uMin = -1.0*_maxJerk;
   // _uMax = _maxJerk;

   _uMin = -1.0*_maxAccel;
   _uMax = _maxAccel;
}

// lower/upper bounds vectors, l, u
void MPCTracker::castMPCToQPConstraintBounds(void)
{   
   // length of states/inputs over _mpcWindow
   // _mpcWindow+1, because x(0) is included
   auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS * _mpcWindow;

   _lowerBounds.resize(2*N_x + N_u,1);
   _lowerBounds.setZero();
   _upperBounds.resize(2*N_x + N_u,1);
   _upperBounds.setZero();

   // Equality bounds
   _lowerBounds.block(0,0, NUM_OF_STATES,1) = -1.0*_current_drone_state;
   _upperBounds.block(0,0, NUM_OF_STATES,1) = -1.0*_current_drone_state;

   // Inequality bounds for the states
   // WARNING Assuming _xMin, _xMax are already set
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _lowerBounds.block(N_x+i*NUM_OF_STATES,0, NUM_OF_STATES,1) = _xMin;
      _upperBounds.block(N_x+i*NUM_OF_STATES,0, NUM_OF_STATES,1) = _xMax;
   }
   // Inequality bounds for the controls, u
   // WARNING Assuming _uMin, _uMax are already set
   for(unsigned int i=0; i<_mpcWindow; i++)
   {
      _lowerBounds.block(2*N_x+i*NUM_OF_INPUTS, 0, NUM_OF_INPUTS,1) = _uMin;
      _upperBounds.block(2*N_x+i*NUM_OF_INPUTS,0, NUM_OF_INPUTS,1) = _uMax;
   }

   if(_debug)
   {
      std::cout<<"Lower bounds vector l = "<<std::endl<< _lowerBounds <<std::endl;
      std::cout<<"Upper bounds vector u = "<<std::endl<< _upperBounds <<std::endl;
   }
}

void MPCTracker::updateQPConstraintsBounds(void)
{
   // Equality for x(0)
   _lowerBounds.block(0,0,NUM_OF_STATES,1) = -1.0*_current_drone_state;
   _upperBounds.block(0,0,NUM_OF_STATES,1) = -1.0*_current_drone_state;

   // auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   // // Equality (in the form of inequality), for x(0)
   // _lowerBounds.block(N_x,0, NUM_OF_STATES,1) = _current_drone_state;
   // _upperBounds.block(N_x,0, NUM_OF_STATES,1) = _current_drone_state;
   if(_debug)
   {
      ROS_INFO("[MPCTracker::updateQPConstraintsBounds] QP bounds are updated");
      std::cout << "[MPCTracker::updateQPConstraintsBounds] Updated lower bound,l = \n " << _lowerBounds << "\n";
      std::cout << "[MPCTracker::updateQPConstraintsBounds] Updated upper bound,u = \n " << _upperBounds << "\n";
   }
}

bool MPCTracker::initQPSolver(void)
{
   ROS_INFO("Initializing QP solver ...");

   _qpSolver.settings()->setVerbosity(_debug);
   _qpSolver.settings()->setWarmStart(true);
   // set the initial data of the QP solver
   _qpSolver.data()->setNumberOfVariables(NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow);
   _qpSolver.data()->setNumberOfConstraints(2*NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow);
   _hessian_sparse = _hessian.sparseView();
   _Ac_sparse = _Ac.sparseView();
   if(!_qpSolver.data()->setHessianMatrix(_hessian_sparse)) return false;
   if(!_qpSolver.data()->setGradient(_gradient)) return false;
   if(!_qpSolver.data()->setLinearConstraintsMatrix(_Ac_sparse)) return false;
   if(!_qpSolver.data()->setLowerBound(_lowerBounds)) return false;
   if(!_qpSolver.data()->setUpperBound(_upperBounds)) return false;

   if(!_qpSolver.initSolver()) return false;


   if(_debug)
   {
      ROS_INFO("QP solver is initialized.");
   }
   return true;

}

bool MPCTracker::initMPCProblem(void)
{
   ros::WallTime startTime = ros::WallTime::now();
   if (_use_6dof_model)
   {
      setTransitionMatrix6DoF(); 
      setInputMatrix6DoF();
      setQ6DoF();
   }
   else
   {
      setTransitionMatrix(); 
      setInputMatrix();
      setQ();
   }
   setR();
   castMPCToQPHessian();
   _current_drone_state.resize(NUM_OF_STATES,1);
   _current_drone_state.setZero();
   _referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_STATES*(_mpcWindow+1),1);
   castMPCToQPGradient();
   castMPCToQPConstraintMatrix();
   if (_use_6dof_model)
      setStateBounds6DoF();
   else
      setStateBounds();
      
   setControlBounds();
   castMPCToQPConstraintBounds();
   if (!initQPSolver())
   {
      ROS_ERROR("[MPCTracker::initMPCProblem] MPC initialization is not successful.");
      return false;
   }

   _is_MPC_initialized = true;

   double total_elapsed = (ros::WallTime::now() - startTime).toSec();
   ROS_INFO("MPC is initialized in %f second(s).", total_elapsed);

   // drone_state_received_ = true; // This is for debugging. SHOULD BE REMOVED
   // target_state_received_ = true; // This is for debugging. SHOULD BE REMOVED
   // mpcLoop();// This is for debugging. SHOULD BE REMOVED
}

bool MPCTracker::updateQP(void)
{
   // update current drone's position (updates QP linear constraints bounds)
   updateQPConstraintsBounds();
   if(!_qpSolver.updateBounds(_lowerBounds, _upperBounds))
   {
      ROS_ERROR("_qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      ROS_INFO("[MPCTracker::updateQP] Bounds are updated.");

   
   // Update the QP gradient vector using  new _referenceTraj
   updateQPGradientVector();
   if(!_qpSolver.updateGradient(_gradient))
   {
      ROS_ERROR("_qpSolver.updateGradient failed to update _gradient");
      return false;
   }
   if(_debug)
      ROS_INFO("QP gradient is updated");

   return true;

}

void MPCTracker::extractSolution(void)
{
   Eigen::VectorXd QPSolution;
   QPSolution = _qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _optimal_state_traj = QPSolution.block(0, 0, NUM_OF_STATES * (_mpcWindow+1), 1);

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS*_mpcWindow;
   _optimal_control_traj = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _mpc_ctrl_sol = _optimal_control_traj.segment(0,NUM_OF_INPUTS);

   _optimal_traj_px.resize(_mpcWindow+1);
   _optimal_traj_py.resize(_mpcWindow+1);
   _optimal_traj_pz.resize(_mpcWindow+1);
   _optimal_traj_vx.resize(_mpcWindow+1);
   _optimal_traj_vy.resize(_mpcWindow+1);
   _optimal_traj_vz.resize(_mpcWindow+1);
   _optimal_traj_ax.resize(_mpcWindow+1);
   _optimal_traj_ay.resize(_mpcWindow+1);
   _optimal_traj_az.resize(_mpcWindow+1);

   _optimal_traj_ux.resize(_mpcWindow);
   _optimal_traj_uy.resize(_mpcWindow);
   _optimal_traj_uz.resize(_mpcWindow);

   _ref_traj_px.resize(_mpcWindow+1);
   _ref_traj_py.resize(_mpcWindow+1);
   _ref_traj_pz.resize(_mpcWindow+1);
   _ref_traj_vx.resize(_mpcWindow+1);
   _ref_traj_vy.resize(_mpcWindow+1);
   _ref_traj_vz.resize(_mpcWindow+1);
   _ref_traj_ax.resize(_mpcWindow+1);
   _ref_traj_ay.resize(_mpcWindow+1);
   _ref_traj_az.resize(_mpcWindow+1);

   _solution_traj_msg.states.resize(_mpcWindow);

   // Update _posehistory_vector for visualiztion
   _posehistory_vector.resize(_mpcWindow+1);
   geometry_msgs::PoseStamped pose_msg;
   auto start_t = ros::Time::now();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      pose_msg.header.frame_id="map";
      pose_msg.header.stamp = start_t + ros::Duration(i*_dt);
      pose_msg.pose.position.x = _optimal_state_traj(i*NUM_OF_STATES+0);
      pose_msg.pose.position.y = _optimal_state_traj(i*NUM_OF_STATES+3);
      pose_msg.pose.position.z = _optimal_state_traj(i*NUM_OF_STATES+6);
      pose_msg.pose.orientation.w=1.0; // Keep 0 rotation
      _posehistory_vector[i] = pose_msg;

      _optimal_traj_px(i) = _optimal_state_traj(i*NUM_OF_STATES+0);
      _optimal_traj_py(i) = _optimal_state_traj(i*NUM_OF_STATES+3);
      _optimal_traj_pz(i) = _optimal_state_traj(i*NUM_OF_STATES+6);

      _optimal_traj_vx(i) = _optimal_state_traj(i*NUM_OF_STATES+1);
      _optimal_traj_vy(i) = _optimal_state_traj(i*NUM_OF_STATES+4);
      _optimal_traj_vz(i) = _optimal_state_traj(i*NUM_OF_STATES+7);

      _optimal_traj_ax(i) = _optimal_state_traj(i*NUM_OF_STATES+2);
      _optimal_traj_ay(i) = _optimal_state_traj(i*NUM_OF_STATES+5);
      _optimal_traj_az(i) = _optimal_state_traj(i*NUM_OF_STATES+8);

      _ref_traj_px(i) = _referenceTraj(i*NUM_OF_STATES+0, 0);
      _ref_traj_py(i) = _referenceTraj(i*NUM_OF_STATES+3, 0);
      _ref_traj_pz(i) = _referenceTraj(i*NUM_OF_STATES+6, 0);

      _ref_traj_vx(i) = _referenceTraj(i*NUM_OF_STATES+1, 0);
      _ref_traj_vy(i) = _referenceTraj(i*NUM_OF_STATES+4, 0);
      _ref_traj_vz(i) = _referenceTraj(i*NUM_OF_STATES+7, 0);

      _ref_traj_ax(i) = _referenceTraj(i*NUM_OF_STATES+2, 0);
      _ref_traj_ay(i) = _referenceTraj(i*NUM_OF_STATES+5, 0);
      _ref_traj_az(i) = _referenceTraj(i*NUM_OF_STATES+8, 0);

      if(i<_mpcWindow)
      {
         _optimal_traj_ux(i) = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         _optimal_traj_uy(i) = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         _optimal_traj_uz(i) = _optimal_control_traj(i*NUM_OF_INPUTS+2);

         // Fill ROS msg
         _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         _solution_traj_msg.states[i].position.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+0 );
         _solution_traj_msg.states[i].position.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+3 );
         _solution_traj_msg.states[i].position.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+6 );
         _solution_traj_msg.states[i].velocity.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+1 );
         _solution_traj_msg.states[i].velocity.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+4 );
         _solution_traj_msg.states[i].velocity.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+6 );
         _solution_traj_msg.states[i].acceleration.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+2 );
         _solution_traj_msg.states[i].acceleration.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+5 );
         _solution_traj_msg.states[i].acceleration.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+8 );
      }
   }

   _solution_traj_msg.header.stamp = start_t;
   _solution_traj_msg.header.frame_id = _reference_frame_id;
   _solution_traj_msg.max_acceleration = _maxAccel. maxCoeff();
   _solution_traj_msg.max_velocity = _maxVel. maxCoeff();

}

void MPCTracker::extractSolution6Dof(void)
{
   Eigen::VectorXd QPSolution;
   QPSolution = _qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _optimal_state_traj = QPSolution.block(0, 0, NUM_OF_STATES * (_mpcWindow+1), 1);

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS*_mpcWindow;
   _optimal_control_traj = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _mpc_ctrl_sol = _optimal_control_traj.segment(0,NUM_OF_INPUTS);

   _optimal_traj_px.resize(_mpcWindow+1);
   _optimal_traj_py.resize(_mpcWindow+1);
   _optimal_traj_pz.resize(_mpcWindow+1);
   _optimal_traj_vx.resize(_mpcWindow+1);
   _optimal_traj_vy.resize(_mpcWindow+1);
   _optimal_traj_vz.resize(_mpcWindow+1);

   _optimal_traj_ux.resize(_mpcWindow);
   _optimal_traj_uy.resize(_mpcWindow);
   _optimal_traj_uz.resize(_mpcWindow);

   _ref_traj_px.resize(_mpcWindow+1);
   _ref_traj_py.resize(_mpcWindow+1);
   _ref_traj_pz.resize(_mpcWindow+1);
   _ref_traj_vx.resize(_mpcWindow+1);
   _ref_traj_vy.resize(_mpcWindow+1);
   _ref_traj_vz.resize(_mpcWindow+1);

   _solution_traj_msg.states.resize(_mpcWindow);

   // Update _posehistory_vector for visualiztion
   _posehistory_vector.resize(_mpcWindow+1);
   geometry_msgs::PoseStamped pose_msg;
   auto start_t = ros::Time::now();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      pose_msg.header.frame_id=_reference_frame_id;
      pose_msg.header.stamp = start_t + ros::Duration(i*_dt);
      pose_msg.pose.position.x = _optimal_state_traj(i*NUM_OF_STATES+0);
      pose_msg.pose.position.y = _optimal_state_traj(i*NUM_OF_STATES+2);
      pose_msg.pose.position.z = _optimal_state_traj(i*NUM_OF_STATES+4);
      pose_msg.pose.orientation.w=1.0; // Keep 0 rotation, for now
      // _posehistory_vector.insert(_posehistory_vector.begin(), pose_msg);
      _posehistory_vector[i] = pose_msg;

      _optimal_traj_px(i) = _optimal_state_traj(i*NUM_OF_STATES+0);
      _optimal_traj_py(i) = _optimal_state_traj(i*NUM_OF_STATES+2);
      _optimal_traj_pz(i) = _optimal_state_traj(i*NUM_OF_STATES+4);

      _optimal_traj_vx(i) = _optimal_state_traj(i*NUM_OF_STATES+1);
      _optimal_traj_vy(i) = _optimal_state_traj(i*NUM_OF_STATES+3);
      _optimal_traj_vz(i) = _optimal_state_traj(i*NUM_OF_STATES+5);

      _ref_traj_px(i) = _referenceTraj(i*NUM_OF_STATES+0, 0);
      _ref_traj_py(i) = _referenceTraj(i*NUM_OF_STATES+2, 0);
      _ref_traj_pz(i) = _referenceTraj(i*NUM_OF_STATES+4, 0);

      _ref_traj_vx(i) = _referenceTraj(i*NUM_OF_STATES+1, 0);
      _ref_traj_vy(i) = _referenceTraj(i*NUM_OF_STATES+3, 0);
      _ref_traj_vz(i) = _referenceTraj(i*NUM_OF_STATES+5, 0);

      if(i<_mpcWindow)
      {
         _optimal_traj_ux(i) = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         _optimal_traj_uy(i) = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         _optimal_traj_uz(i) = _optimal_control_traj(i*NUM_OF_INPUTS+2);

         // Fill ROS msg
         _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         _solution_traj_msg.states[i].position.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+0 );
         _solution_traj_msg.states[i].position.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+2 );
         _solution_traj_msg.states[i].position.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+4 );
         _solution_traj_msg.states[i].velocity.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+1 );
         _solution_traj_msg.states[i].velocity.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+3 );
         _solution_traj_msg.states[i].velocity.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+5 );
         _solution_traj_msg.states[i].acceleration.x = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         _solution_traj_msg.states[i].acceleration.y = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         _solution_traj_msg.states[i].acceleration.z = _optimal_control_traj(i*NUM_OF_INPUTS+2);
      }
   }

   _solution_traj_msg.header.stamp = start_t;
   _solution_traj_msg.header.frame_id = _reference_frame_id;
   _solution_traj_msg.max_acceleration = _maxAccel. maxCoeff();
   _solution_traj_msg.max_velocity = _maxVel. maxCoeff();

}

bool MPCTracker::mpcLoop(void)
{
   ros::WallTime startTime = ros::WallTime::now();

   if(!_is_MPC_initialized)
   {
      ROS_WARN("MPC controller is not initialized. Skipping MPC loop.");
      return false;
   }
   if(!_drone_state_received)
   {
      ROS_WARN("[MPC controller] Drone state is not received yet. Skipping MPC loop.");
      return false;
   }

   // Update gradient and bounds
   if(!updateQP())
   {
      ROS_ERROR("[mpcLoop] Failed to update bounds and gradient");
      return false;
   }
   
   // Solve MPC
   if(!_qpSolver.solve())
   {
      ROS_ERROR("MPC solution is not found");
      return false;
   }

   if(_debug)
   {
      double total_elapsed = (ros::WallTime::now() - startTime).toSec();
      ROS_INFO("MPC is solved in %f second(s).", total_elapsed);
   }
}

bool MPCTracker::engageMPCCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
   _is_mpc_engaged = req.data;
   if(_is_mpc_engaged)
      res.message = "MPC controller is engaged";
   else
      res.message = "MPC controller is disengaged";

   res.success = 0;
   
   return true;
}

void MPCTracker::droneImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   // WARNING The following is WRONG!!!!
   // TODO: Need to transform IMU from body frame to local frame, and remove gravity magnitude from z axis
   // Get acceleration values
   _current_drone_accel.setZero();
   _current_drone_accel << msg->linear_acceleration.x , msg->linear_acceleration.y , msg->linear_acceleration.z;
}

void MPCTracker::droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   if(!_drone_state_received)
      _drone_state_received = true;

   _drone_state_current_t = msg->header.stamp;
   _current_drone_state.setZero();
   // TODO Sync time stamps of _current_drone_accel with pose, before adding it to _current_drone_state
   //  state order: [px, vx, ax, py, vy, ay, pz, vz, az]
   //  state order for 6DoF: [px, vx, py, vy, pz, vz]

   if (_use_6dof_model)
   {
      _current_drone_state(0,0) = msg->pose.pose.position.x;
      _current_drone_state(1,0) = msg->twist.twist.linear.x;

      _current_drone_state(2,0) = msg->pose.pose.position.y;
      _current_drone_state(3,0) = msg->twist.twist.linear.y;

      _current_drone_state(4,0) = msg->pose.pose.position.z;
      _current_drone_state(5,0) = msg->twist.twist.linear.z;
   }
   else
   {
      _current_drone_state(0,0) = msg->pose.pose.position.x;
      _current_drone_state(1,0) = msg->twist.twist.linear.x;
      _current_drone_state(2,0) = _current_drone_accel(0);

      _current_drone_state(3,0) = msg->pose.pose.position.y;
      _current_drone_state(4,0) = msg->twist.twist.linear.y;
      _current_drone_state(5,0) = _current_drone_accel(1);

      _current_drone_state(6,0) = msg->pose.pose.position.z;
      _current_drone_state(7,0) = msg->twist.twist.linear.z;
      _current_drone_state(8,0) = _current_drone_accel(2);
   }
}

void MPCTracker::refTrajCallback(const custom_trajectory_msgs::StateTrajectory::ConstPtr& msg)
{
   // WARNING The rate of MPC is affected by
   // the rate of Odom (drone state) (default 30Hz from mavros/local_position/odom),
   // and _referenceTraj
   // and the size of MPC problem
   // The MPC rate will be close to the min(odom, _referenceTraj, MPC execution time)

   // Make sure we have a new reference trajectory
   auto dt = (msg->header.stamp - _ref_traj_last_t).toSec();
   if (dt <= 0.0)
   {
      ROS_ERROR("[MPCTracker::refTrajCallback] Received an old reference trajectory");
      return;
   }
   _ref_traj_last_t = msg->header.stamp;

   // Make sure we have a new drone state measurement
   dt = (_drone_state_current_t - _drone_state_last_t).toSec();
   if (dt <= 0.0)
   {
      ROS_ERROR("[MPCTracker::refTrajCallback] Received an old drone state. Return");
      return;
   }
   _drone_state_last_t = _drone_state_current_t;

   // Make sure we have enough state predictions of the target
   if (msg->states.size() < (_mpcWindow+1) )
   {
      ROS_ERROR("[MPCTracker::refTrajCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg->states.size(), _mpcWindow+1);
      return;
   }

   // Update _referenceTraj
   _referenceTraj.setZero();
   for (int i=0; i<_mpcWindow+1; i++)
   {
      if (_use_6dof_model)
      {
         _referenceTraj(i*NUM_OF_STATES+0,0) = msg->states[i].position.x;
         _referenceTraj(i*NUM_OF_STATES+1,0) = msg->states[i].velocity.x;

         _referenceTraj(i*NUM_OF_STATES+2,0) = msg->states[i].position.y;
         _referenceTraj(i*NUM_OF_STATES+3,0) = msg->states[i].velocity.y;

         _referenceTraj(i*NUM_OF_STATES+4,0) = msg->states[i].position.z;
         _referenceTraj(i*NUM_OF_STATES+5,0) = msg->states[i].velocity.z;
      }
      else
      {
         _referenceTraj(i*NUM_OF_STATES+0,0) = msg->states[i].position.x;
         _referenceTraj(i*NUM_OF_STATES+1,0) = msg->states[i].velocity.x;
         _referenceTraj(i*NUM_OF_STATES+2,0) = msg->states[i].acceleration.x;

         _referenceTraj(i*NUM_OF_STATES+3,0) = msg->states[i].position.y;
         _referenceTraj(i*NUM_OF_STATES+4,0) = msg->states[i].velocity.y;
         _referenceTraj(i*NUM_OF_STATES+5,0) = msg->states[i].acceleration.y;

         _referenceTraj(i*NUM_OF_STATES+6,0) = msg->states[i].position.z;
         _referenceTraj(i*NUM_OF_STATES+7,0) = msg->states[i].velocity.z;
         _referenceTraj(i*NUM_OF_STATES+8,0) = msg->states[i].acceleration.z;
      }
   }

   /* Solve MPC problem ! */
   if(!mpcLoop())
   {
      ROS_ERROR("[MPCTracker::refTrajCallback] Error in mpcLooop");
      return;
   }

   // Extract solutions, updates _optimal_state_traj, _optimal_control_traj, _mpc_ctrl_sol
   if(_use_6dof_model)
      extractSolution6Dof();
   else
      extractSolution();

   // Publish desired trajectory, visualization, ... etc
   if(_pub_pose_path)
   {
      pubPoseHistory();
   }

   // Publish optimal trajectory
   _desired_traj_pub.publish(_solution_traj_msg);
   // Publish first control solution u[0] to the geometric controller
   pubMultiDofTraj();
}

void MPCTracker::testCasesCallback(const std_msgs::Empty::ConstPtr& msg)
{
   if(_run_test_cases)
   {
      testCases();
   }
}

void MPCTracker::appendPoseHistory(geometry_msgs::PoseStamped pose_msg)
{
   _posehistory_vector.insert(_posehistory_vector.begin(), pose_msg);
  if (_posehistory_vector.size() > _mpcWindow+1) {
    _posehistory_vector.pop_back();
  }
}

void MPCTracker::pubPoseHistory(void)
{
   nav_msgs::Path msg;

   msg.header.stamp = _posehistory_vector[0].header.stamp;
   msg.header.frame_id = _reference_frame_id;
   msg.poses = _posehistory_vector;

   _poseHistory_pub.publish(msg);
}

void MPCTracker::testCases(void)
{
   if (_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Starting test Case. Target is hovering at 1.0m altitude. Follower is at rest on the ground.");
   }
   if(_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Setting the reference trajectory");
   }
   
   // For timing this function
   auto t1 = ros::Time::now();

   // Create a _referenceTraj of a target hovering at 1.0m altitude from ground
   _referenceTraj.resize(NUM_OF_STATES*(_mpcWindow+1),1);
   _referenceTraj.setZero();
   for (int i=0; i<_mpcWindow+1; i++)
   {
      if (_use_6dof_model)
      {
         // states order: [px, vx, py, vy, pz, vz]
         _referenceTraj(i*NUM_OF_STATES+4,0) = 1.0; // z coordinate at all times
         _referenceTraj(i*NUM_OF_STATES+0,0) = 10.0; // x coordinate at all time
      }
      else
      {
         // states order: [px, vx, ax, py, vy, ay, pz, vz, az]
         _referenceTraj(i*NUM_OF_STATES+6,0) = 1.0; // z coordinate at all times
         _referenceTraj(i*NUM_OF_STATES+0,0) = 10.0; // x coordinate at all times  
      }
   }

   if(_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Setting the drone state");
   }
   // Current drone state
   _current_drone_state.setZero();
   if (_use_6dof_model)
   {
      // state order: [px, vx, py, vy, pz, vz]
      _current_drone_state(0,0)=0.1; // px
      _current_drone_state(2,0)=-0.5; // py
      _current_drone_state(4,0)=0.1; // pz
   }
   else
   {
      // state order: [px, vx, ax, py, vy ,ay, pz, vz, az]
      _current_drone_state(0,0)=0.1; // px
      _current_drone_state(3,0)=-0.5; // py
      _current_drone_state(6,0)=0.1; // pz
   }


   if(_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Updating QP gradient and bounds");
   }
   
   // Update QP
   if(!updateQP())
   {
      ROS_ERROR("[MPCTracker::testCase] Failed to update bounds and/or gradient");
      return;
   }
   
   // Solve MPC
   if(_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Solving QP");
   }
   if(!_qpSolver.solve())
   {
      ROS_ERROR("MPC solution is not found");
      return;
   }

   if(_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Extracting solution");
   }
   // Extract solutions, updates _optimal_state_traj, _optimal_control_traj, _mpc_ctrl_sol
   if (_use_6dof_model)
      extractSolution6Dof();
   else
      extractSolution();

   _desired_traj_pub.publish(_solution_traj_msg);

   if(_debug)
   {
      std::cout << "[MPCTracker::testCase] Optimal control: " << std::endl << _mpc_ctrl_sol << std::endl;
      std::cout << "[MPCTracker::testCase] Optimal state trajectory: " << std::endl << _optimal_state_traj << std::endl;
      std::cout << "[MPCTracker::testCase] Optimal control trajectory: " << std::endl << _optimal_control_traj << std::endl;
   }

   // Apply the optimal control inputs to the initial condition
   auto x0 = _current_drone_state;
   for (int i=0; i < _mpcWindow; i++)
   {
      x0 = _A*x0 + _B*_optimal_control_traj.segment(i*NUM_OF_INPUTS, NUM_OF_INPUTS);
   }

   // Publish desired trajectory, visualization, ... etc
   if(_pub_pose_path)
   {
      if(_debug)
      {
         ROS_INFO("[MPCTracker::testCase] Publishing predicted position path");
      }
      pubPoseHistory();
   }

   if (_debug)
   {
      ROS_INFO("[MPCTracker::testCase] Test case is Done!");
   }

   ROS_INFO(" Test case took %f seconds.", (ros::Time::now()-t1).toSec());

   // Compare the simulated satate to the one computed by the optimization
   auto x_N_opt = _optimal_state_traj.segment(NUM_OF_STATES*_mpcWindow, NUM_OF_STATES);
   std::cout << "[MPCTracker::testCase] Simulated state at time step t= " << _mpcWindow << "\n" << x0 << "\n";
   std::cout << "[MPCTracker::testCase] Optimal state at time step t= " << _mpcWindow << "\n" << x_N_opt << "\n";
   std::cout << "[MPCTracker::testCase] Error between simulated and optimal final state at t= " << _mpcWindow << "\n" << (x_N_opt-x0).norm() << "\n";

   if(_save_mpc_data)
   {
      saveMPCDataToFile();
   }

   if(_plot)
      plotSolutions();
}

void MPCTracker::saveMPCDataToFile(void)
{
   //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
   const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
   const static Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");
   std::ofstream file(_outputCSVFile);
   if (file.is_open())
   {
      std::string sep= "\n------------------------------------------\n";

      file << "Initial state, x(0): \n";
      file << _current_drone_state.format(CSVFormat) << sep ;

      file << " A : \n";
      file << _A.format(CleanFmt) << sep;
      
      file << "B : \n";
      file << _B.format(CleanFmt) << sep;

      file << "Q : \n";
      file << _Q.format(CleanFmt) << sep;

      file << "R : \n";
      file << _R.format(CleanFmt) << sep;

      file << "Hessian matrix, P: \n";
      file << _hessian.format(CleanFmt) << sep;

      file << "Constarints matrix, Ac: \n";
      file << _Ac.format(CleanFmt) << sep;

      file << "Lower bounds, l: \n";
      file << _lowerBounds.format(CleanFmt) << sep;

      file << "Upper bounds, l: \n";
      file << _upperBounds.format(CleanFmt) << sep;

      file << "gradient, q: \n";
      file << _gradient.format(CleanFmt) << sep;

      file << "Optima state trajectory, X: \n";
      file << _optimal_state_traj.format(CleanFmt) << sep;

      file << "Optimal contorl trajectory, U: \n";
      file << _optimal_control_traj.format(CleanFmt) << sep;

      file.close();
      ROS_INFO("[MPCTracker::saveMPCDataToFile] Saved MPC solutions to file: %s", _outputCSVFile.c_str());
   }
   else
      ROS_ERROR("[MPCTracker::saveMPCDataToFile] Coudl not open file %s", _outputCSVFile.c_str());
}

void MPCTracker::plotSolutions(void)
{
   // Plot only if _run_test_cases==true
   if(!_run_test_cases)
   {
      ROS_ERROR("[MPCTracker::plotSolutions] Plotting is only allowed in offline test cases. Real-time plotting is done in RViz");
      return;
   }

   // time vector
   Eigen::VectorXd times(_mpcWindow+1);
   times.setLinSpaced(_mpcWindow+1, 0, _dt*_mpcWindow);
   
   plotty::figure();

   // subplots
   plotty::subplot(3, 3, 1);   // state_px, ref_px
   plotty::labelPlot("Optimal solution", times, _optimal_traj_px);
   plotty::labelPlot("Reference trajectory", times, _ref_traj_px, "--");
   plotty::title("x-position");
   plotty::legend();

   plotty::subplot(3, 3, 2);   // state_py, ref_py
   plotty::plot(times, _optimal_traj_py);
   plotty::plot(times, _ref_traj_py, "--");
   plotty::title("y-position");

   plotty::subplot(3, 3, 3);   // state_pz, ref_pz
   plotty::plot(times, _optimal_traj_pz);
   plotty::plot(times, _ref_traj_pz, "--");
   plotty::title("z-position");

   plotty::subplot(3, 3, 4);
   plotty::plot(times, _optimal_traj_vx);
   plotty::plot(times, _ref_traj_vx, "--");
   plotty::title("x-velocity");

   plotty::subplot(3, 3, 5);
   plotty::plot(times, _optimal_traj_vy);
   plotty::plot(times, _ref_traj_vy, "--");
   plotty::title("y-velocity");

   plotty::subplot(3, 3, 6);
   plotty::plot(times, _optimal_traj_vz);
   plotty::plot(times, _ref_traj_vz, "--");
   plotty::title("z-velocity");

   if (_use_6dof_model)
   {
      plotty::subplot(3, 3, 7);
      plotty::plot(times.segment(0, _mpcWindow), _optimal_traj_ux);
      plotty::title("x-acceleration");

      plotty::subplot(3, 3, 8);
      plotty::plot(times.segment(0, _mpcWindow), _optimal_traj_uy);
      plotty::title("y-acceleration");

      plotty::subplot(3, 3, 9);
      plotty::plot(times.segment(0, _mpcWindow), _optimal_traj_uz);
      plotty::title("z-acceleration");
   }
   else
   {
      plotty::subplot(3, 3, 7);
      plotty::plot(times, _optimal_traj_ax);
      plotty::plot(times, _ref_traj_ax, "--");
      plotty::title("x-acceleration");

      plotty::subplot(3, 3, 8);
      plotty::plot(times, _optimal_traj_ay);
      plotty::plot(times, _ref_traj_ay, "--");
      plotty::title("y-acceleration");

      plotty::subplot(3, 3, 9);
      plotty::plot(times, _optimal_traj_az);
      plotty::plot(times, _ref_traj_az, "--");
      plotty::title("z-acceleration");
   }

   plotty::show();
}

void MPCTracker::pubMultiDofTraj(void)
{
   trajectory_msgs::MultiDOFJointTrajectory msg;
   msg.header.frame_id = _reference_frame_id;
   msg.header.stamp = ros::Time::now();

   trajectory_msgs::MultiDOFJointTrajectoryPoint point;
   geometry_msgs::Transform pos; // position
   geometry_msgs::Twist vel, acc;

   pos.translation.x = _optimal_traj_px(1);
   pos.translation.y = _optimal_traj_py(1);
   pos.translation.z = _optimal_traj_pz(1);

   vel.linear.x = _optimal_traj_vx(1);
   vel.linear.y = _optimal_traj_vy(1);
   vel.linear.z = _optimal_traj_vz(1);

   if (_use_6dof_model)
   {
      acc.linear.x = _optimal_traj_ux(0);
      acc.linear.y = _optimal_traj_uy(0);
      acc.linear.z = _optimal_traj_uz(0);
   }
   else
   {
      acc.linear.x = _optimal_traj_ax(1);
      acc.linear.y = _optimal_traj_ay(1);
      acc.linear.z = _optimal_traj_az(1);
   }

   // Minimum altitude constraints, useful for the drone not to hit the ground!
   // if (pos.translation.z < _minAltitude)
   // {
   //    pos.translation.z = _minAltitude;
   //    vel.linear.z = 0.0;
   //    acc.linear.z = 0.0;
   // }

   point.transforms.push_back(pos);
   point.velocities.push_back(vel);
   point.accelerations.push_back(acc);

   msg.points.push_back(point);

   _multiDofTraj_pub.publish(msg);

}
/*
********************************************************************
                       !! REMOVE !! Commander class                             
********************************************************************
*/


Commander::Commander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
takeoff_alt_(2.0),
hold_pos_(false)
{
   nh_private.param("takeoff_alt", takeoff_alt_, 2.0);
   nh_private.param("commander_rate", cmd_rate_, 0.05);

   mavstateSub_ = nh_.subscribe("mavros/state", 1, &Commander::mavstateCallback, this,ros::TransportHints().tcpNoDelay());

   setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

   arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
   set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
   armAndOffb_service_ =  nh_.advertiseService("mpc_commander/arm_and_offboard", &Commander::armAndOffbCallback, this);
   takeoff_service_ = nh_.advertiseService("mpc_commander/takeoff", &Commander::takeoffCallback, this);
   holdPos_service_ = nh_.advertiseService("mpc_commander/hold", &Commander::holdPosCallback, this);
   
   cmdloop_timer_ = nh_.createTimer(ros::Duration(cmd_rate_), &Commander::cmdloopCallback, this); // Define timer for constant loop rate
}

Commander::~Commander(){ /** Desturctor */ }

bool Commander::armAndOffbCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   mavros_msgs::CommandBool arm_cmd;
   mavros_msgs::SetMode offb_set_mode;
   arm_cmd.request.value = true;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   if( arming_client_.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
   }
   if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      ROS_INFO("Offboard enabled");
   }
   return true;
}

bool Commander::holdPosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   holdPose();
}

bool Commander::takeoffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   last_sp_update_req_ = ros::Time::now();

   setpoint_msg_.header.stamp = ros::Time::now();
   setpoint_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
   setpoint_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + 
                              mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ + 
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   setpoint_msg_.yaw = 0.0; // TODO: ??
   setpoint_msg_.position.x = dronePose_msg_.pose.position.x;
   setpoint_msg_.position.y = dronePose_msg_.pose.position.y;
   setpoint_msg_.position.z = dronePose_msg_.pose.position.z + takeoff_alt_;

   mavros_msgs::CommandBool arm_cmd;
   mavros_msgs::SetMode offb_set_mode;
   arm_cmd.request.value = true;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   if( arming_client_.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
   }
   if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      ROS_INFO("Offboard enabled");
   }

   ROS_INFO("Position setpoint is updated for takeoff");
   
   return true;
}

void Commander::mavstateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void Commander::setDronePose(geometry_msgs::PoseStamped msg)
{
   dronePose_msg_ = msg;
}

void Commander::updateSetpoint(mavros_msgs::PositionTarget msg)
{
   last_sp_update_req_ = ros::Time::now();
   setpoint_msg_ = msg;
}

void Commander::holdPose(void)
{
   setpoint_msg_.header.stamp = ros::Time::now();
   setpoint_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
   setpoint_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + 
                              mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ + 
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   setpoint_msg_.yaw = 0.0; // TODO: ??
   setpoint_msg_.position.x = dronePose_msg_.pose.position.x;
   setpoint_msg_.position.y = dronePose_msg_.pose.position.y;
   setpoint_msg_.position.z = dronePose_msg_.pose.position.z;
}

void Commander::publishSetpoint(void)
{
   // if( (ros::Time::now() - last_sp_update_req_) > ros::Duration(2.0))
   // {
   //    ROS_WARN_THROTTLE(1, "Setpoint is not updated externally. Holding at current position.");
   //    if(!hold_pos_)
   //    {
   //       holdPose();
   //       hold_pos_ = true;
   //    }
   // }
   // else
   // {
   //    hold_pos_ = false;
   // }
   
   setpoint_pub_.publish(setpoint_msg_);
}

void Commander::cmdloopCallback(const ros::TimerEvent& event)
{
   publishSetpoint();
}