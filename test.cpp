#include "robot_arm_6dof_kinematics.h"
#include <iostream>
#include "Actuator_Handler.h"
#include <chrono>
#include <thread>

int main() {
    using namespace Robotics;
    using namespace Actuator;

    Kinematics_6DOF_RobotArm RAK;
    Kinematics_6DOF_RobotArm::JointConstraints jc {{
        {-M_PI_2, M_PI_2}, {0, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI_2, M_PI_2}, {-M_PI, M_PI} 
    }};

    RAK.setConstraints(jc);

    SMS_STS st { "/dev/ttyACM0", BaudRate::r_1M };
    Actuator_Handler<6> AH {
        std::array<ServoMotor, 6> {
            ServoMotor {
                11, // ID
                ServoCalibration {
                    2014,   // correction
                    { 2014-1024, 2014+1024 }, // target angle limit
                    true   // posDirection
                },
                st
            },
            ServoMotor {
                12, // ID
                ServoCalibration {
                    2028,   // correction
                    { 1000, 2030},
                    true   // posDirection
                },
                st
            },
            ServoMotor {
                13, // ID
                ServoCalibration {
                    3103,   // correction
                    { 2000, 3300},
                    true   // posDirection
                },
                st
            },
            ServoMotor {
                14, // ID
                ServoCalibration {
                    2062,   // correction
                    { 1030, 3100},
                    true   // posDirection
                },
                st
            },
            ServoMotor {
                15, // ID
                ServoCalibration {
                    2074,   // correction
                    { 1900, 3100},
                    true   // posDirection
                },
                st
            },
            ServoMotor {
                16, // ID
                ServoCalibration {
                    2047,   // correction
                    { 0, 4095},
                    true   // posDirection
                },
                st
            },





        },
        st
    };

    Kinematics_6DOF_RobotArm::Pose pose { {130, 0, 100}, {0.8660254, .5, 0, 0} };
    auto solutions { RAK.solveAll(pose) };

    std::cout << "Number of solution: " << solutions.size() << '\n';
    for (auto& solution: solutions) {
        // Kinematics_6DOF_RobotArm::IKSolution solution { solutions[0] };
        // std::cout << solution.joint_angles << '\n';
        std::cout << solution.joint_angles << '\n';
    }

    Kinematics_6DOF_RobotArm::IKSolution solution { solutions[0] };
    // AH.setAngle({0, 1, 2, 3}, {solution.joint_angles[0], solution.joint_angles[1], solution.joint_angles[2], solution.joint_angles[3]});
    // AH.setAngle({0, 1, 2}, {solution.joint_angles[0], solution.joint_angles[1], M_PI_4});
    // AH.setAngle(solution.joint_angles);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // AH.updateAllFeedback();
    // auto angles { AH.getAllAngle() };
    // for ( const double angle : angles ) {
    //     std::cout << angle << ' ';
    // }
    // std::cout << '\n'; 
    // AH.disableTorque();

    return 0;
}

