/**
 * @file ArmRobotKinematics.hpp
 * 
 * @brief Defines the ArmRobotKinematics class, which performs forward and inverse kinematic calculations
 *        of a robot based on its geometry. The class uses a Jacobian pseudoinverse method for inverse
 *        kinematics iteratively and includes an empty method for implementing an algebraic solution specific
 *        to a robot's geometry.
 * 
 * This header file contains the definition of the ArmRobotKinematics class, including its member variables,
 * methods, and any necessary includes or definitions.
 * 
 * @author Ryan Barry
 * @date Created: August 24, 2024
 */


#ifndef ARM_ROBOT_KINEMATICS_HPP
#define ARM_ROBOT_KINEMATICS_HPP

#include <vector>
#include "Frame.hpp"
#include "Pose.hpp"
#include "KinematicParameters.hpp"
#include <Eigen/Dense>



class ArmRobotKinematics{
public:
    ArmRobotKinematics(vector<Frame>* frames) : m_frames(frames) {}


    // Compute forward kinematics of the robot based on it's current joint angles.
    void forwardKinematics();

    /**
    * @brief a Jacobian pseudo-inverse iterative method to calculate the inverse kinematic solution of the robot
    * 
    * @param target_pos The desired end-effector position [x, y, z]
    * 
    * @param target_orient The desired end-effector orientation [roll, pitch, yaw]
    * 
    * @return A vector of doubles corresponding to the joint angle
    */
    vector<double> inverseKinematics(Pose target_pose); 

    // Get the current pose
    Pose getPose() { return m_pose; }

    // Setter functions for tolerances (inline definitions)
    void setPositionTolerance(double tolerance) { m_params.posTolerance = tolerance; }
    void setOrientationTolerance(double tolerance) { m_params.orTolerance = tolerance; }
    void setMaxIterations(double maxIter) { m_params.maxIterations = maxIter; }
    void setIncrement(double increment) { m_params.increment = increment; }
    void setMomentum(double momentum) { m_params.momentum = momentum; }

    // Update the joint values of the robot
    vector<double> updateJointValues();

private:
    Eigen::MatrixXd m_jacobian; ///< The Jacobian of the robot
    Eigen::MatrixXd m_jacobianPseudoInverse; ///< The pseudo inverse of the Jacobian matrix
    void jacobian(); ///< Calculate the Jacobian matrix

    vector<Frame>* m_frames; ///< A pointer to the robot's frames
    Eigen::Matrix4d m_T; ///< The robot's transform matrix from the base to the hand
    Pose m_pose; ///< The robot's end effector pose

    KinematicParameters m_params;   // The tolerances for iterative inverse kinematics

    vector<double> m_jointValues; ///< The robot's joint values
};

#endif