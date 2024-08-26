#include "ArmRobot.hpp"
#include <iostream>
using namespace std;

int main(){
    ArmRobot robot = ArmRobot();
    robot.addFrame(REVOLUTE, 0.0, 0.0, 4, 0.0);
    robot.addFrame(REVOLUTE, 0.0, 0.0, 1, 0.0);
    robot.addFrame(REVOLUTE, 0.0, 0.0, 2, 0.0);
    // robot.moveJoint(0, 0.001);
    // robot.moveJoint(1, M_PI/2);
    // robot.moveJoint(2, -M_PI/2);
    robot.kinematics.forwardKinematics();
    vector<double> jointValues = robot.kinematics.getJointValues();
    Pose pose = robot.kinematics.getPose();
    cout << "Initial Pose: " << endl;
    cout << "x: " << pose.x << " y: " << pose.y << " z: " << pose.z << endl;
    cout << "roll: " << pose.roll << " pitch: " << pose.pitch << " yaw: " << pose.yaw << endl;
    cout << "Initial Joint Values: " << endl;
    pose.x = 7;
    pose.y = 0.007;
    pose.z = 0;
    pose.roll = 0;
    pose.pitch = 0;
    pose.yaw = 0.001;

    jointValues = robot.kinematics.inverseKinematics(pose);
    for(auto value : jointValues){
        cout << value << " ";
    }
}