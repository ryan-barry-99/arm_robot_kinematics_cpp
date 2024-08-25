#ifndef KINEMATIC_PARAMETERS_HPP
#define KINEMATIC_PARAMETERS_HPP

#include <cmath>

class KinematicParameters{
public:
    // Tolerance values for Jacobian pseudo-inverse iterative inverse kinematics
    double posTolerance = 0.002;  ///< The position tolerance
    double orTolerance = 2*M_PI/180;   ///< The orientation tolerance
    double maxIterations = 10000; ///< The max number of iterations to try to converge on a solution
    double increment = 0.0001;   ///< The increment to move each joint
    double momentum = 0.1;    ///< For optimization
};

#endif