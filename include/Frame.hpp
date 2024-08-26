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
#include <iostream>

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
        JointType jointType, 
        double thetaFix, 
        double d, 
        double a, 
        double alphaFix
    )
        :   m_jointType(jointType),
            m_thetaFix(thetaFix),
            m_theta(0),
            m_d(d),
            m_a(a),
            m_alphaFix(alphaFix),
            m_alpha(0)
    {}

    /**
     * @brief Creates a 4x4 transformation matrix based on provided parameters.
     * 
     * @return Eigen::Matrix4d The resulting transformation matrix
     */
    Eigen::Matrix4d getTransformationMatrix(){
        double ct = cos(m_thetaFix + m_theta);
        double st = sin(m_thetaFix + m_theta);
        double ca = cos(m_alphaFix + m_alpha);
        double sa = sin(m_alphaFix + m_alpha);


        m_transformMatrix <<    ct, -st * ca, st * sa, m_a * ct,
                                st, ct * ca, -ct * sa, m_a * st,
                                0, sa, ca, m_d,
                                0, 0, 0, 1;

        return m_transformMatrix;
    }

    int getJointType() { return m_jointType; }


    /**
     * @brief Moves the joint by a specified value.
     * 
     * @param jointValue The value to move the joint by.
     */
    void moveJoint(double jointValue){
        // std::cout << "Moving joint by: " << jointValue << std::endl;
        if (m_jointType == REVOLUTE) {
            m_theta = fmod(jointValue, 2 * M_PI);
            m_jointValue = m_theta;
            if(m_theta > M_PI){
                m_jointValue -= 2*M_PI;
            }
        } 
        else if (m_jointType == PRISMATIC) {
            if (m_d == 0) {
                m_a += jointValue;
                m_jointValue = m_a;
            } else if (m_a == 0) {
                m_d += jointValue;
                m_jointValue = m_d;
            }
        }
        // std::cout << "Joint value: " << m_jointValue << std::endl;
    }

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
