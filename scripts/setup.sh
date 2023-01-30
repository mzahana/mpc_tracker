#!/bin/bash

# This scripts sets up the control package of the autonomous drone hunter project
# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com

echo "Setting up the mpc_tracker package..."

# SUDO_PASS=$1
# GIT_TOKEN=$2

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
echo $SUDO_PASS | sudo -S apt-get install -y libyaml-cpp-dev libtool libtool-bin
else
    sudo apt-get install -y libyaml-cpp-dev libtool libtool-bin
fi

if [ -z "$CATKIN_WS" ]; then
	CATKIN_WS=$HOME/catkin_ws
	echo "path to catkin_ws is defined at $CATKIN_WS" && echo    
fi

# Dependencies for mavros_controllers and trajectory generation
if [ ! -d "$CATKIN_WS/src/catkin_simple" ]; then
    cd $CATKIN_WS/src
    git clone https://github.com/catkin/catkin_simple
else
    cd $CATKIN_WS/src/catkin_simple && git pull
fi

ETHZ_PKG="eigen_catkin mav_comm eigen_checks glog_catkin nlopt geodetic_utils mav_trajectory_generation plotty waypoint_navigator"
for p in $ETHZ_PKG; do
    if [ ! -d "$CATKIN_WS/src/$p" ]; then
        cd $CATKIN_WS/src
        git clone https://github.com/ethz-asl/$p
    else
        cd $CATKIN_WS/src/$p && git pull
    fi
done

# waypoint_navigator: Change branch
cd $CATKIN_WS/src/waypoint_navigator && git checkout fix/abort_path

# Clone Systemtrio packages
PKGS="trajectory_prediction custom_trajectory_msgs"
for p in $PKGS; do
    if [ ! -d "$CATKIN_WS/src/$p" ]; then
        echo "Didn't find $p. Cloning it..."
        cd $CATKIN_WS/src
        git clone https://${GIT_USER}:${GIT_TOKEN}@github.com/mzahana/$p
    else
        echo "$p is found. Pulling latest code..."
        cd $CATKIN_WS/src/$p && git pull
    fi
done



# mavros_controllers (geometric controller SE(3)/SO(3) )
if [ ! -d "$CATKIN_WS/src/mavros_controllers" ]; then
    cd $CATKIN_WS/src
    git clone https://github.com/Jaeyoung-Lim/mavros_controllers
else
    cd $CATKIN_WS/src/mavros_controllers && git pull
fi



if [ ! -d "$HOME/src" ]; then
    echo "Creating $HOME/src ..." && echo
    mkdir -p $HOME/src
fi

# osqp
if [ -z "$OSQP_SRC" ]; then
	OSQP_SRC=$HOME/src
	echo "path to catkin_ws is defined at $OSQP_SRC" && echo    
fi

if [ ! -d "$OSQP_SRC/osqp" ]; then
    cd $OSQP_SRC
    git clone --recursive https://github.com/osqp/osqp.git
else
    cd $OSQP_SRC/osqp && git pull
fi
cd $OSQP_SRC/osqp && rm -rf build
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
if [ -z "$SUDO_PASS" ]; then
    cmake --build . --target install
else
    echo $SUDO_PASS | sudo -S cmake --build . --target install
fi

# osqqp-eigen: https://github.com/robotology/osqp-eigen
if [ ! -d "$OSQP_SRC/osqp-eigen" ]; then
    cd $OSQP_SRC
    git clone https://github.com/robotology/osqp-eigen.git
else
    cd $OSQP_SRC/osqp-eigen && git pull
fi
cd $OSQP_SRC/osqp-eigen
rm -rf build
mkdir build && mkdir install
cd build
# cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$HOME/src/osqp-eigen/install ../
cmake -DCMAKE_BUILD_TYPE=Release ../
make
if [ -z "$SUDO_PASS" ]; then
    make install
else
    echo $SUDO_PASS | sudo -S make install
fi
echo "export OsqpEigen_DIR=$OSQP_SRC/osqp-eigen/install" >> ${HOME}/.bashrc
#grep -xF 'export OsqpEigen_DIR=$HOME/src/osqp-eigen/install' ${HOME}/.bashrc || echo "export OsqpEigen_DIR=\$HOME/src/osqp-eigen/install" >> ${HOME}/.bashrc
source $HOME/.bashrc

# Matplotlib, required for plotting
pip install matplotlib

# Build catkin_ws
cd $CATKIN_WS && catkin build
source $HOME/.bashrc
