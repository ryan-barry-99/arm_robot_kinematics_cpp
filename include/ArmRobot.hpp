/**
 * @file ArmRobot.hpp
 * 
 * @brief Defines the ArmRobot class, which extends ArmRobotKinematics to represent a specific robotic arm configuration.
 *        This class sets up the robot's structure using frames defined by joint type and physical parameters such as 
 *        length and rotation. The robot is defined using a series of frames added in the constructor.
 * 
 * @details The ArmRobot class is an example of how to configure a robotic arm with a combination of revolute and prismatic joints. 
 *          It initializes the arm's structure using the addFrame method, with specific parameters provided in the constructor.
 *          The class inherits from ArmRobotKinematics to leverage kinematics calculations.
 * 
 * @author Ryan Barry
 * @date Created: August 25, 2024
 */

#ifndef ARM_ROBOT_HPP
#define ARM_ROBOT_HPP

#include "ArmRobotKinematics.hpp"
#include <cmath>  // Include cmath for constants like M_PI

/**
 * @class ArmRobot
 * 
 * @brief Represents a robotic arm with a predefined configuration of joints and links.
 * 
 * This class inherits from ArmRobotKinematics to utilize kinematic functions and adds specific frames
 * representing the robot's joints and links in its constructor. The robot configuration includes both 
 * revolute and prismatic joints with fixed lengths and angles.
 */
class ArmRobot : public ArmRobotKinematics {
public:
    /**
     * @brief Constructs an ArmRobot instance and defines its configuration.
     * 
     * The constructor sets up the robot's joint and link configuration using the addFrame method.
     * It specifies parameters for each frame, including joint type (REVOLUTE or PRISMATIC), fixed angles, 
     * lengths, and displacements. These parameters are based on a given example and can be modified to fit 
     * specific robot designs.
     */
    ArmRobot() {
        // Define the configuration of the robot using the addFrame method
        // The parameters correspond to the Python code provided
        
        // Example frames - replace these constants with actual values
        const double LINK1_LENGTH = 1.0;   ///< Example length for link1
        const double LINK2_LENGTH = 1.0;   ///< Example length for link2
        const double WRIST_LENGTH = 0.5;   ///< Example length for wrist
        const double LINK4_LENGTH = 0.3;   ///< Example length for link4

        // Adding frames with specific parameters
        addFrame(REVOLUTE, M_PI, 0.0, LINK1_LENGTH, 0.0);  ///< Link 1 - Revolute joint
        addFrame(REVOLUTE, 0.0, 0.0, LINK2_LENGTH, 0.0);   ///< Link 2 - Revolute joint
        addFrame(REVOLUTE, 0.0, 0.0, WRIST_LENGTH, 0.0);   ///< Link 3 - Revolute joint
        addFrame(PRISMATIC, 0.0, LINK4_LENGTH, 0.0, 0.0);  ///< Link 4 - Prismatic joint
    }
};

#endif // ARM_ROBOT_HPP
