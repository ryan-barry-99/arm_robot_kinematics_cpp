# ArmRobot Library

## Overview

The `ArmRobot` library provides tools for performing kinematic calculations for robotic arms. It extends the `ArmRobotKinematics` class to allow for customizable robotic arm configurations. This library supports both forward and inverse kinematics calculations using a Jacobian pseudoinverse method for iterative solutions.

Find the Python version of this repository [here](https://github.com/ryan-barry-99/arm_robot_kinematics_py).

Note: The C++ version of this repository is still in active development as I hash out some convergence issues with the Inverse Kinematics

## Features

- **Forward Kinematics**: Compute the end-effector position and orientation based on joint parameters.
- **Inverse Kinematics**: Determine joint angles/positions required to achieve a desired end-effector pose using iterative methods.
- **Flexible Configuration**: Define the robot configuration either in the constructor or externally using the `addFrame` method.

## Dependencies

- **Eigen3**: The library relies on Eigen3 for matrix operations and linear algebra. Ensure Eigen3 is installed and properly configured in your build environment.

## Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/ryan-barry-99/arm_robot_kinematics_cpp.git
   cd your-repository
