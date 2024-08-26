/**
 * @file Frame.hpp
 * 
 * @brief This header file defines the Frame class, which represents a single frame in an arm robot. 
 * It encapsulates the properties and methods related to a robot's frame, including the joint type, 
 * geometric parameters, and transformation matrix calculation.
 * 
 * The Frame class includes:
 * - Member variables for joint type, DH parameters (theta, d, a, alpha), and transformation matrix.
 * - Methods for manipulating joint values and creating transformation matrices.
 * 
 * @author Ryan Barry
 * @date Created: August 24, 2024
 */

#ifndef FRAME_HPP
#define FRAME_HPP

#include <Eigen/Dense>
#include <cmath> 

using namespace std;

// Enum for different types of joints in the robot arm
enum JointType {
    PRISMATIC,
    REVOLUTE,
    FIXED_TRANSLATION,
    FIXED_ROTATION
};

class Frame {
public:
    /**
     * @brief Constructs a Frame object with specified joint parameters.
     * 
     * @param jointType Type of the joint (e.g., PRISMATIC, REVOLUTE, etc.)
     * @param thetaFix Fixed angle theta_n+1 to align x_n with x_n+1
     * @param d Distance d_n+1 along the z_n axis to make x_n collinear with x_n+1
     * @param a Distance a_n+1 along the rotated x_n axis
     * @param alphaFix Angle alpha_n+1 to rotate the z_n axis to align with z_n+1
     */
    Frame(
        JointType jointType,    ///< Type of the joint (e.g., PRISMATIC, REVOLUTE, etc.)
        double thetaFix,    ///< Fixed angle theta_n+1 to align x_n with x_n+1
        double d,          ///< Distance d_n+1 along the z_n axis
        double a,          ///< Distance a_n+1 along the rotated x_n axis
        double alphaFix   ///< Angle alpha_n+1 to rotate z_n axis to align with z_n+1
    );

    /**
     * @brief Creates a 4x4 transformation matrix based on provided parameters.
     * 
     * @return Eigen::Matrix4d The resulting transformation matrix
     */
    Eigen::Matrix4d createTransformationMatrix();

    int getJointType() { return m_jointType; }

    Eigen::Matrix4d getTransformMatrix() { return m_transformMatrix; }

    /**
     * @brief Moves the joint by a specified value.
     * 
     * @param jointValue The value to move the joint by.
     */
    void moveJoint(double jointValue);

    double getTheta() { return m_theta; }
    double getD() { return m_d; }
    double getJointValue() { return m_jointValue; }

private:
    int m_jointType;    ///< Type of the joint
    double m_theta;     ///< Rotate about the z_n axis an angle theta_n+1 to make x_n parallel to x_n+1
    double m_thetaFix;  ///< Fixed angle theta_n+1
    double m_d;         ///< Distance d_n+1 along the z_n axis
    double m_a;         ///< Distance a_n+1 along the x_n axis
    double m_alpha;     ///< Rotate the z_n axis about the x_n+1 axis an angle of alpha_n+1 to align the z_n axis with the z_n+1 axis
    double m_alphaFix;  ///< Angle alpha_n+1 to align z_n with z_n+1
    double m_jointValue;  ///< The current value of the joint
    Eigen::Matrix4d m_transformMatrix; ///< Transformation matrix for the frame
};

#endif
