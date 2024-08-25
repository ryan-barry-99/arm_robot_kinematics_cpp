/**
 * @file Pose.hpp
 * 
 * @brief Defines the Pose class, which represents the position and orientation of a robotic end effector
 *        in 3D space. This class includes attributes for the X, Y, and Z positions as well as the roll, pitch,
 *        and yaw angles to fully describe the pose of the end effector.
 * 
 * This header file provides the definition of the Pose class, including its member variables that capture
 * both the translational and rotational aspects of the end effector's pose in space.
 * 
 * @class Pose
 * 
 * @details The Pose class is a simple container for storing and manipulating the position (x, y, z) and orientation
 *          (roll, pitch, yaw) of a robotic end effector. It is used within kinematic calculations to represent
 *          the target or current state of the robot's end effector.
 * 
 * @author Ryan Barry
 * @date Created: August 24, 2024
 */

#ifndef POSE_H
#define POSE_H

class Pose {
public:
    double x;     ///< The X position of the end effector.
    double y;     ///< The Y position of the end effector.
    double z;     ///< The Z position of the end effector.
    double roll;  ///< The roll (rotation around the X-axis) of the end effector.
    double pitch; ///< The pitch (rotation around the Y-axis) of the end effector.
    double yaw;   ///< The yaw (rotation around the Z-axis) of the end effector.
};

#endif