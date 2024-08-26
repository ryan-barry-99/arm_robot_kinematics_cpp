/**
 * @file KinematicParameters.hpp
 * 
 * @brief Defines the KinematicParameters class, which contains parameters for controlling the behavior of 
 *        iterative inverse kinematics algorithms, such as the Jacobian pseudo-inverse method. The parameters 
 *        include tolerances for position and orientation errors, maximum iterations, step increments, and momentum.
 * 
 * This header file provides the definition of the KinematicParameters class, outlining various parameters 
 * that influence the convergence and accuracy of inverse kinematics solutions for robotic arms.
 * 
 * @class KinematicParameters
 * 
 * @details The KinematicParameters class encapsulates various settings used during the computation of inverse 
 *          kinematics. These settings determine the convergence criteria and step sizes for iterative solvers, 
 *          allowing fine-tuning of the algorithm’s performance and precision.
 * 
 * @author Ryan Barry
 * @date Created: August 24, 2024
 */

#ifndef KINEMATIC_PARAMETERS_HPP
#define KINEMATIC_PARAMETERS_HPP

#include <cmath>

class KinematicParameters {
public:
    double posTolerance = 0.005;           ///< The position tolerance for convergence.
    double orTolerance = 1 * M_PI / 180;   ///< The orientation tolerance for convergence (in radians).
    double maxIterations = 10000000;          ///< The maximum number of iterations allowed to converge to a solution.
    double increment = 0.01;             ///< The increment step size for adjusting each joint during iteration.
    double momentum = 0.1;                 ///< Momentum factor to optimize the convergence speed.
    double epsilon = 1e-5;                 ///< A small value to prevent division by zero.
    double dampingFactor = 0.1;            ///< Damping factor for the Damped Least Squares (DLS) method.
};

#endif
