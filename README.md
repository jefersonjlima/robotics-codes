# Robotics Codes

Principal repository https://gitlab.com/cursoseaulas/robotica-movel/-/wikis/home

## Sumary

1. **Robot Kinematics and Dynamics Modeling**
	1. [Rotation Matrix](src/examples/1/rotation_ex.cpp)
	1. [Translation Matrix](src/examples/1/translation_ex.cpp)
	1. [Homogeneous Transformation Matrix](src/examples/1/hTransformation_ex.cpp)
	1. [Differential Drive Kinematics Model](src/examples/1/diff_drive_Kinematics_ex.cpp)
	1. [Dynamics Modeling](src/examples/1/dc_motor_ex.cpp)
1. **Locomotion and Perception**
	1. Perception e Sensors in Modern Robots
	1. Bayes Filter Review
	1. The Kalman Filter
	1. The Extended Kalman Filter
	1. Understanding Quaternions
1. **Robotic Simulation**
	1. Introduction to ROS - Robotics Operating System
1. **Robot Control**
	1. Classic Control - PID Review
	1. Modern Control - LQR and MCP Control
1. **Simultaneous localization and mapping (SLAM)**
	1. Iteratively Reweighted Least Squares
	1. The Iterative Closest Point (ICP) Algorithm
	1. k-nearest neighbors algorithm
	1. Random Sample Consensus
	1. Adaptive Monte Carlo localization (AMCL)
	1. Visual Odometry
1. **Robot as a Service (RaaS)**
	1. An Intro to Blockchain
	1. Smart Contracts and Robotics
	1. Introduction to EVM and Solidity
	1. How to deploy a Robot Smart Contract
1. **Mobile Robotics Lecture Notes**

## Prepare Environment

```bash
$ sudo apt-get update 
$ sudo apt install git cmake build-essential libeigen3-dev libboost-all-dev -y
$ pip install matplotlib
$ git clone https://github.com/lava/matplotlib-cpp.git /tmp/matplotlibcpp
$ cd /tmp/matplotlibcpp/ && make build && cd build
$ cmake .. && sudo make install
```
If you are using WSL-Ubuntu

```bash
sudo ln -s /home/${USER}/.local/lib/python3.8/site-packages/numpy/core/include/numpy /usr/include/numpy
sudo apt-get install python3-gi-cairo python3-tk
```

## Usage

```bash
$ git clone https://gitlab.com/cursoseaulas/robotics-codes.git
$ cd robotics-codes && make build
# next you can try to run the example
$ ./build/bin/<EXAMPLE>_ex
```

## References

* Roland Siegwart, Illah Reza Nourbakhsh, Davide Scaramuzza. **Introduction to Autonomous Mobile Robots** (Intelligent Robotics and Autonomous Agents series), 2nd ed. The MIT Press, 2011.
* Sebastian Thrun, Wolfram Burgard, Dieter Fox. **Probabilistic Robotics** (Intelligent Robotics and Autonomous Agents series). The MIT Press, 2005.
* Ulrich Nehmzow. **Mobile Robotics: A Practical Introduction**, 2nd ed. Springer, 2003.

## Bugs & Feature Requests

Please report bugs and request features using the [issues](https://gitlab.com/cursoseaulas/robotics-codes/-/issues)
