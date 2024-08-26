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
#include <iostream>



class ArmRobotKinematics{
public:
    ArmRobotKinematics(vector<Frame>* frames) : m_frames(frames) {}

    // Get the current pose
    Pose getPose() { return m_pose; }

    // Setter functions for tolerances (inline definitions)
    void setPositionTolerance(double tolerance) { m_params.posTolerance = tolerance; }
    void setOrientationTolerance(double tolerance) { m_params.orTolerance = tolerance; }
    void setMaxIterations(double maxIter) { m_params.maxIterations = maxIter; }
    void setIncrement(double increment) { m_params.increment = increment; }
    void setMomentum(double momentum) { m_params.momentum = momentum; }

    // Compute forward kinematics of the robot based on it's current joint angles.
    void forwardKinematics(){
        m_T = Eigen::Matrix4d::Identity();  // Initialize transformation matrix as identity

        for (auto frame : *m_frames) {
            m_T = m_T * frame.createTransformationMatrix();
        }

        m_pose.x = m_T(0, 3);
        m_pose.y = m_T(1, 3);
        m_pose.z = m_T(2, 3);

        m_pose.roll = std::atan2(m_T(2, 1), m_T(2, 2));
        m_pose.pitch = std::atan2(-m_T(2, 0), std::sqrt(m_T(2, 1) * m_T(2, 1) + m_T(2, 2) * m_T(2, 2)));
        m_pose.yaw = std::atan2(m_T(1, 0), m_T(0, 0));
    }

    /**
    * @brief a Jacobian pseudo-inverse iterative method to calculate the inverse kinematic solution of the robot
    * 
    * @param target_pos The desired end-effector position [x, y, z]
    * 
    * @param target_orient The desired end-effector orientation [roll, pitch, yaw]
    * 
    * @return A vector of doubles corresponding to the joint angle
    */
    vector<double> inverseKinematics(Pose target_pose){

        // Target position and orientation
        Eigen::Vector3d target_position(target_pose.x, target_pose.y, target_pose.z);
        Eigen::Vector3d target_orientation(target_pose.roll, target_pose.pitch, target_pose.yaw);

        // Initialize variables
        Eigen::Vector3d position_error = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector3d orientation_error = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_frames->size());
        size_t iterations = 0;
        Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(m_frames->size());

        // While error is greater than tolerance, iterate
        while ((position_error.norm() > m_params.posTolerance || orientation_error.norm() > m_params.orTolerance) 
                && iterations < m_params.maxIterations){

            this->forwardKinematics();
            
            // Get the current position and orientation
            Eigen::Vector3d current_position(m_pose.x, m_pose.y, m_pose.z);
            Eigen::Vector3d current_orientation(m_pose.roll, m_pose.pitch, m_pose.yaw);

            // Calculate position error
            Eigen::Vector3d position_error = target_position - current_position;

            // Calculate orientation error
            Eigen::Vector3d orientation_error = target_orientation - current_orientation;

            this->jacobian();

            // Calculate condition number of the Jacobian matrix
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_jacobian);
            double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

            double scaled_increment = m_params.increment / cond;

            // Compute the concatenated error vector and scale it
            Eigen::VectorXd concatenated_error(6);
            concatenated_error.head<3>() = position_error * scaled_increment;
            concatenated_error.tail<3>() = orientation_error * scaled_increment;

            // Solve for joint increments (dq)
            dq = m_jacobianPseudoInverse * concatenated_error + m_params.momentum * prev_dq;

            // Update previous dq
            prev_dq = dq;

            // Apply joint increments to update joint angles
            int i=0;
            for(auto frame : *m_frames){
                frame.moveJoint(frame.getTheta() + dq(i));
                i++;
            }
            
            iterations++;
            // Check if maximum iterations are reached and throw an error if so
            if (iterations >= m_params.maxIterations) {
                throw std::runtime_error("Inverse kinematics did not converge.");
            }
        }

        return this->getJointValues();
    } 


    // Update the joint values of the robot
    vector<double> getJointValues(){
        m_jointValues.clear();
        for(auto frame: *m_frames){
            m_jointValues.push_back(frame.getJointValue());
        }
        return m_jointValues;
    }

private:
    Eigen::MatrixXd m_jacobian; ///< The Jacobian of the robot
    Eigen::MatrixXd m_jacobianPseudoInverse; ///< The pseudo inverse of the Jacobian matrix

    vector<Frame>* m_frames; ///< A pointer to the robot's frames
    Eigen::Matrix4d m_T; ///< The robot's transform matrix from the base to the hand
    Pose m_pose; ///< The robot's end effector pose

    KinematicParameters m_params;   // The tolerances for iterative inverse kinematics

    vector<double> m_jointValues; ///< The robot's joint values
    
    ///< Calculate the Jacobian matrix 
    void jacobian(){
        // Initialize Jacobian matrix with size 6 x number of frames
        Eigen::MatrixXd J(6, m_frames->size());
        J.setZero();  // Initialize to zero

        this->forwardKinematics();

        Eigen::Vector3d On(m_pose.x, m_pose.y, m_pose.z); // Position of the end-effector in the base frame coordinate system

        int i=0;
        for(auto frame : *m_frames){
            int jointType = frame.getJointType();

            if(jointType == FIXED_ROTATION || jointType == FIXED_TRANSLATION){
                continue;
            }

            Eigen::Matrix4d T = frame.getTransformMatrix();
            
            // Get the rotation axis of the i-th joint (third column)
            Eigen::Vector3d Zi = T.col(2).head<3>();
            
            // Get the origin of the i-th frame (fourth column)
            Eigen::Vector3d Oi = T.col(3).head<3>();

            if(jointType == REVOLUTE){
                // Linear velocity for revolute joint
                J.block<3, 1>(0, i) = Zi.cross(On - Oi);
                // Angular velocity for revolute joint
                J.block<3, 1>(3, i) = Zi;
            }
            else if(jointType == PRISMATIC){
                // Linear velocity for prismatic joint
                J.block<3, 1>(0, i) = Zi;
                // Angular velocity for prismatic joint is zero
                J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
            }
            else {
                throw std::invalid_argument("Unknown joint type");
            }

            i++;
        }
        m_jacobian = J;
        // Compute the pseudo-inverse of J
        Eigen::MatrixXd m_jacobianPseudoInverse = J.completeOrthogonalDecomposition().pseudoInverse();
    }

};

#endif