#include "ArmRobot.hpp"


ArmRobot::ArmRobot() : kinematics(ArmRobotKinematics(&m_frames)) {
    // Users can define their robot configuration here if they prefer
    // For example:
    // addFrame(REVOLUTE, M_PI, 0.0, LINK1_LENGTH, 0.0);
    // addFrame(REVOLUTE, 0.0, 0.0, LINK2_LENGTH, 0.0);
    // addFrame(REVOLUTE, 0.0, 0.0, WRIST_LENGTH, 0.0);
    // addFrame(PRISMATIC, 0.0, LINK4_LENGTH, 0.0, 0.0);
}

Frame* ArmRobot::addFrame(
        JointType jointType,
        double thetaFix,  
        double d,         
        double a,         
        double alphaFix   
    ) {
    // Use emplace_back to construct Frame in place
    m_frames.emplace_back(jointType, thetaFix, d, a, alphaFix);
    // Return a pointer to the newly added Frame
    // Note: No direct way to get the pointer, so we use reference to the last added frame
    return &m_frames.back();
}