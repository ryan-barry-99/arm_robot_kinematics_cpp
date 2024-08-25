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
    ArmRobotKinematics() : m_frames() {}

    // Add a frame to the robot arm
    Frame* addFrame(
        JointType jointType,    ///< Type of the joint (e.g., PRISMATIC, REVOLUTE, etc.)
        double thetaFix,    ///< Fixed angle theta_n+1 to align x_n with x_n+1
        double d,          ///< Distance d_n+1 along the z_n axis
        double a,          ///< Distance a_n+1 along the rotated x_n axis
        double alphaFix   ///< Angle alpha_n+1 to rotate z_n axis to align with z_n+1
    );

    // Compute forward kinematics
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

private:
    Eigen::MatrixXd m_jacobian; ///< The Jacobian of the robot
    Eigen::MatrixXd m_jacobianPseudoInverse; ///< The pseudo inverse of the Jacobian matrix
    void jacobian();

    vector<Frame> m_frames;
    Eigen::Matrix4d m_T; ///< The robot's transform matrix from the base to the hand
    Pose m_pose; ///< The robot's end effector pose
    int m_numFrames = 0;

    KinematicParameters m_params;   // The tolerances for iterative inverse kinematics


};

#endif