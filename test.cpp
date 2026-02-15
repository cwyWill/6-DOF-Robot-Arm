#include "robot_arm_6dof_kinematics.h"
#include <iostream>

int main() {
    using namespace Robotics;
    Kinematics_6DOF_RobotArm RAK;

    Kinematics_6DOF_RobotArm::Pose pose { {150, 0, 160}, {0, 1, 0, 0} };
    auto solutions { RAK.solveAll(pose) };

    if ( !solutions.empty() ) {
        Kinematics_6DOF_RobotArm::IKSolution solution { solutions[0] };
        std::cout << solution.joint_angles << '\n';
    }
    std::cout << "Number of solution: " << solutions.size() << '\n';


    return 0;
}

