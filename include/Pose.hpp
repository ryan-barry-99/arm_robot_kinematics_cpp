#ifndef POSE_H
#define POSE_H

class Pose{
public:
    double x;     ///< The X position of the end effector
    double y;     ///< The Y position of the end effector
    double z;     ///< The Z position of the end effector
    double roll;  ///< The roll of the end effector
    double pitch; ///< The pitch of the end effector
    double yaw;   ///< The yaw of the end effector
};

#endif