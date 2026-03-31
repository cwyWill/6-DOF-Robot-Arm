#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/joint_angle_array.hpp"

#include "Actuator_Handler.h"

using joint_angle_array = interfaces::msg::JointAngleArray;

using namespace std::chrono_literals;

class ActuationNode : public rclcpp::Node
{
public:
    ActuationNode() :
        Node("actuation_node"),
        AH { std::array<ServoMotor, 6> {
                ServoMotor {
                    11, // ID
                    ServoCalibration {
                        2014,   // correction
                        { 2014-1024, 2014+1024 }, // target angle limit
                        false   // posDirection
                    },
                    st
                },
                ServoMotor {
                    12, // ID
                    ServoCalibration {
                        2028,   // correction
                        { 200, 2030},
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
                        false   // posDirection
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
                        false   // posDirection
                    },
                    st
                },
            },
            st
        }
    {

        // Subscriber
        auto command_callback = [this](const joint_angle_array& msg) -> void {
            std::array<double, 6> target_angles;
            for (size_t i = 0; i < 6; ++i) {
                target_angles[i] = msg.angles[i];
            }
            AH.setTargetAngles(target_angles);
        };
        joint_angle_subscription_ = this->create_subscription<joint_angle_array>("joint_angle_topic", 10, command_callback);

        // Publisher
        auto timer_callback = [this]() -> void {
            AH.updateAllFeedback();
            auto current_angles = AH.getAllAngle();
            auto joint_angle_msg = joint_angle_array();
            for (size_t i = 0; i < 6; ++i) {
                joint_angle_msg.angles[i] = current_angles[i];
            }
            current_joint_angle_publisher_->publish(joint_angle_msg);
            RCLCPP_INFO(this->get_logger(), "Current joint angles: [%f, %f, %f, %f, %f, %f]", current_angles[0], current_angles[1], current_angles[2], current_angles[3], current_angles[4], current_angles[5]);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::Subscription<joint_angle_array>::SharedPtr joint_angle_subscription_;
    rclcpp::Publisher<joint_angle_array>::SharedPtr current_joint_angle_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    SMS_STS st { "/dev/ttyACM0", BaudRate::r_1M };
    Actuator_Handler<6> AH;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
