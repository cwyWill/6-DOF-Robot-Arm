#include "robot_arm_6dof_kinematics.h"
#include <iostream>

int main() {
    using namespace Robotics;
    Kinematics_6DOF_RobotArm RAK;

    Kinematics_6DOF_RobotArm::JointConstraints jc {{
        {-M_PI_2, M_PI_2}, {0, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI, M_PI} 
    }};
    RAK.setConstraints(jc);

    Kinematics_6DOF_RobotArm::Pose pose { {150, 0, 120}, {0, 1, 0, 0} };
    auto solutions { RAK.solveAll(pose) };

    std::cout << "Number of solution: " << solutions.size() << '\n';
    for (auto& solution: solutions) {
        // Kinematics_6DOF_RobotArm::IKSolution solution { solutions[0] };
        // std::cout << solution.joint_angles << '\n';
        std::cout << solution.joint_angles << '\n';
    }


    return 0;
}

