#include "ArmRobot.hpp"
#include <iostream>
using namespace std;

int main(){
    ArmRobot robot = ArmRobot();
    robot.addFrame(REVOLUTE, 0.0, 0.0, 4.32, 0.0);
    robot.addFrame(REVOLUTE, 0.0, 0.0, 1, 0.0);
    robot.moveJoint(0, M_PI/2);
    robot.kinematics.forwardKinematics();
    vector<double> jointValues = robot.kinematics.getJointValues();
    Pose pose = robot.kinematics.getPose();
    cout << "Initial Pose: " << endl;
    cout << "x: " << pose.x << " y: " << pose.y << " z: " << pose.z << endl;
    cout << "roll: " << pose.roll << " pitch: " << pose.pitch << " yaw: " << pose.yaw << endl;
    cout << "Initial Joint Values: " << endl;
    for(auto value : jointValues){
        cout << value << " ";
    }
}