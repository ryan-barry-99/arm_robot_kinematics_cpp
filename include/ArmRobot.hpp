/**
 * @file ArmRobot.hpp
 * 
 * @brief Defines the ArmRobot class, which extends the ArmRobotKinematics class to provide
 *        a customizable robotic arm implementation for kinematic calculations.
 * 
 * This class allows users to define the configuration of their robotic arm. The constructor is
 * intentionally left empty to provide flexibility. Users can either configure their robot directly
 * in the constructor or instantiate the class and configure it using the addFrame method externally.
 * 
 * Example usage:
 * @code
 * ArmRobot robot;
 * robot.addFrame(REVOLUTE, M_PI, 0.0, LINK1_LENGTH, 0.0);
 * robot.addFrame(REVOLUTE, 0.0, 0.0, LINK2_LENGTH, 0.0);
 * robot.addFrame(REVOLUTE, 0.0, 0.0, WRIST_LENGTH, 0.0);
 * robot.addFrame(PRISMATIC, 0.0, LINK4_LENGTH, 0.0, 0.0);
 * @endcode
 * 
 * @author Ryan Barry
 * @date Created: August 25, 2024
 */

#ifndef ARM_ROBOT_HPP
#define ARM_ROBOT_HPP

#include "ArmRobotKinematics.hpp"
#include "Frame.hpp"
#include <cmath>  // Include cmath for constants like M_PI

/**
 * @class ArmRobot
 * @brief Represents a robotic arm that performs kinematic calculations.
 * 
 * This class inherits from ArmRobotKinematics and uses its methods to perform forward and inverse
 * kinematics calculations. The constructor is empty, allowing users to define their robot configuration
 * either directly in the constructor or by using the addFrame method externally.
 */
class ArmRobot{
public:
    /**
     * @brief Default constructor for the ArmRobot class.
     * 
     * This constructor is intentionally left empty. Users can either:
     * - Define their robot configuration directly in this constructor by calling the addFrame method.
     * - Instantiate the ArmRobot class and configure the robot using the addFrame method externally.
     * 
     * Example of defining robot configuration externally:
     * @code
     * ArmRobot robot;
     * robot.addFrame(REVOLUTE, M_PI, 0.0, LINK1_LENGTH, 0.0);
     * robot.addFrame(REVOLUTE, 0.0, 0.0, LINK2_LENGTH, 0.0);
     * robot.addFrame(REVOLUTE, 0.0, 0.0, WRIST_LENGTH, 0.0);
     * robot.addFrame(PRISMATIC, 0.0, LINK4_LENGTH, 0.0, 0.0);
     * @endcode
     */
    ArmRobot() : kinematics(ArmRobotKinematics(&m_frames)) {
        // Users can define their robot configuration here if they prefer
        // For example:
        // addFrame(REVOLUTE, M_PI, 0.0, LINK1_LENGTH, 0.0);
        // addFrame(REVOLUTE, 0.0, 0.0, LINK2_LENGTH, 0.0);
        // addFrame(REVOLUTE, 0.0, 0.0, WRIST_LENGTH, 0.0);
        // addFrame(PRISMATIC, 0.0, LINK4_LENGTH, 0.0, 0.0);
    }

    ArmRobotKinematics kinematics; ///< The kinematics object for the robot

    // Add a frame to the robot arm
    Frame* addFrame(
        JointType jointType,    ///< Type of the joint (e.g., PRISMATIC, REVOLUTE, etc.)
        double thetaFix,    ///< Fixed angle theta_n+1 to align x_n with x_n+1
        double d,          ///< Distance d_n+1 along the z_n axis
        double a,          ///< Distance a_n+1 along the rotated x_n axis
        double alphaFix   ///< Angle alpha_n+1 to rotate z_n axis to align with z_n+1
    ) {
        // Use emplace_back to construct Frame in place
        m_frames.emplace_back(jointType, thetaFix, d, a, alphaFix);
        // Return a pointer to the newly added Frame
        // Note: No direct way to get the pointer, so we use reference to the last added frame
        return &m_frames.back();
    }

    void moveJoint(int frameIndex, double jointValue){
        m_frames[frameIndex].moveJoint(jointValue);
    }
    
private:
    std::vector<Frame> m_frames; ///< Vector of frames that make up the robot arm
};

#endif // ARM_ROBOT_HPP
