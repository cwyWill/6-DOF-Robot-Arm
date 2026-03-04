#include "robot_arm_6dof_kinematics.h"
#include <iostream>
#include <chrono>
#include <thread>


int main() {

    using namespace Robotics;

    Kinematics_6DOF_RobotArm RAK;
    Kinematics_6DOF_RobotArm::JointConstraints jc {{
        {-M_PI_2-.1, M_PI_2+.1},
        {-M_PI, 0.},
        {-M_PI*2./3.-.1, M_PI/12.+.1},
        {-M_PI_2-.1, M_PI_2+.1},
        {-M_PI/6.-.1, (M_PI)*2./3.+.1},
        {-M_PI-.1, M_PI+.1} 
    }};
    RAK.setConstraints(jc);

    Kinematics_6DOF_RobotArm::Quaternion quat { 0, -1, 0, 0 };
    // Kinematics_6DOF_RobotArm::Quaternion quat { cos(.872), 0, sin(.872), 0 };
    // quat = quat * Kinematics_6DOF_RobotArm::Quaternion { 0, 1, 0, 0 };
    std::cout << "quat: " << quat.w() << ' ' << quat.x() << ' ' << quat.y() << ' ' << quat.z() << '\n';


    Kinematics_6DOF_RobotArm::Pose pose { {130, 0, 150}, quat};
    auto solutions { RAK.solveAll(pose) };

    std::cout << "Number of solution: " << solutions.size() << '\n';
    for (auto& solution: solutions) {
        // Kinematics_6DOF_RobotArm::IKSolution solution { solutions[0] };
        // std::cout << solution.joint_angles << '\n';
        std::cout << solution.joint_angles << '\n';
    }

    return 0;
}
