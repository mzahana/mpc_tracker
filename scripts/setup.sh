#!/bin/bash

# This scripts sets up the control package of the autonomous drone hunter project
# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com

echo "Setting up the mpc_tracker package..."

SUDO_PASS=$2
GIT_TOKEN=$1

# For coloring terminal output
RED='\033[0;31m'
NC='\033[0m' # No Color

# if [ -z "${SUDO_PASS}" ]; then
#     echo -e "${RED} SUDO_PASS environment variable is not defined ${NC}" && echo
#     exit 10
# fi

# if [ -z "${GIT_TOKEN}" ]; then
#     echo -e "${RED} GIT_TOKEN environment variable is not defined ${NC}" && echo
#     exit 10
# fi

# Trajectory generation dependencies
if [ ! -z "${SUDO_PASS}" ]; then
echo $SUDO_PASS | sudo -S apt-get install -y libyaml-cpp-dev    
else
    sudo apt-get install -y libyaml-cpp-dev
fi



# Dependencies for mavros_controllers and trajectory generation
if [ ! -d "$HOME/catkin_ws/src/catkin_simple" ]; then
    cd $HOME/catkin_ws/src
    git clone https://github.com/catkin/catkin_simple
else
    cd $HOME/catkin_ws/src/catkin_simple && git pull
fi

ETHZ_PKG="eigen_catkin mav_comm eigen_checks glog_catkin nlopt geodetic_utils mav_trajectory_generation plotty waypoint_navigator"
for p in $ETHZ_PKG; do
    if [ ! -d "$HOME/catkin_ws/src/$p" ]; then
        cd $HOME/catkin_ws/src
        git clone https://github.com/ethz-asl/$p
    else
        cd $HOME/catkin_ws/src/$p && git pull
    fi
done

# waypoint_navigator: Change branch
cd $HOME/catkin_ws/src/waypoint_navigator && git checkout fix/abort_path

# Clone Systemtrio packages
PKGS="trajectory_prediction custom_trajectory_msgs"
for p in $PKGS; do
    if [ ! -d "$HOME/catkin_ws/src/$p" ]; then
        echo "Didn't find $p. Cloning it..."
        cd $HOME/catkin_ws/src
        git clone https://${GIT_TOKEN}@github.com/SystemTrio-Robotics/$p
    else
        echo "$p is found. Pulling latest code..."
        cd $HOME/catkin_ws/src/$p && git pull
    fi
done



# mavros_controllers (geometric controller SE(3)/SO(3) )
if [ ! -d "$HOME/catkin_ws/src/mavros_controllers" ]; then
    cd $HOME/catkin_ws/src
    git clone https://github.com/Jaeyoung-Lim/mavros_controllers
else
    cd $HOME/catkin_ws/src/mavros_controllers && git pull
fi



if [ ! -d "$HOME/src" ]; then
    echo "Creating $HOME/src ..." && echo
    mkdir -p $HOME/src
fi

# osqp
if [ ! -d "$HOME/src/osqp" ]; then
    cd $HOME/src
    echo $SUDO_PASS | sudo -S git clone --recursive https://github.com/osqp/osqp.git
else
    cd $HOME/src/osqp && git pull
fi
cd $HOME/src/osqp 
echo $SUDO_PASS | sudo -S rm -rf build
echo $SUDO_PASS | sudo -S mkdir build
cd build
echo $SUDO_PASS | sudo -S cmake -G "Unix Makefiles" ..
echo $SUDO_PASS | sudo -S cmake --build .
if [ -z "$SUDO_PASS" ]; then
    echo $SUDO_PASS | sudo -S cmake --build . --target install
else
    echo $SUDO_PASS | sudo -S cmake --build . --target install
fi

# osqqp-eigen: https://github.com/robotology/osqp-eigen
if [ ! -d "$HOME/src/osqp-eigen" ]; then
    cd $HOME/src
    echo $SUDO_PASS | sudo -S git clone https://github.com/robotology/osqp-eigen.git
else
    cd $HOME/src/osqp-eigen && git pull
fi
cd $HOME/src/osqp-eigen
echo $SUDO_PASS | sudo -S rm -rf build
echo $SUDO_PASS | sudo -S mkdir build && mkdir install
cd build
# cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$HOME/src/osqp-eigen/install ../
echo $SUDO_PASS | sudo -S cmake -DCMAKE_BUILD_TYPE=Release ../
echo $SUDO_PASS | sudo -S make
if [ -z "$SUDO_PASS" ]; then
    echo $SUDO_PASS | sudo -S make install
else
    echo $SUDO_PASS | sudo -S make install
fi
grep -xF 'export OsqpEigen_DIR=$HOME/src/osqp-eigen/install' ${HOME}/.bashrc || echo "export OsqpEigen_DIR=\$HOME/src/osqp-eigen/install" >> ${HOME}/.bashrc
source $HOME/.bashrc

# Matplotlib, required for plotting
pip install matplotlib

# Build catkin_ws
cd $HOME/catkin_ws && catkin build
source $HOME/.bashrc
