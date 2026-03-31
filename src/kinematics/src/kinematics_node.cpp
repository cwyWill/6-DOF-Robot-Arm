#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/joint_angle_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <Eigen/Dense>
#include "robot_arm_6dof_kinematics.h"

using namespace std::chrono_literals;

using Pose = geometry_msgs::msg::Pose;
using Quaternion = geometry_msgs::msg::Quaternion;

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node")
    {
        // Publisher
        joint_angle_publisher_ = this->create_publisher<interfaces::msg::JointAngleArray>("joint_angle_topic", 10);
        // Set joint constraints for the robot arm
        Robotics::Kinematics_6DOF_RobotArm::JointConstraints jc {{
            {-M_PI_2-.1, M_PI_2+.1},
            {-M_PI, 0.},
            {-M_PI*2./3.-.1, M_PI/12.+.1},
            {-M_PI_2-.1, M_PI_2+.1},
            {-M_PI/6.-.1, (M_PI)*2./3.+.1},
            {-M_PI-.1, M_PI+.1} 
        }};
        RAK.setConstraints(jc);

        // function callback
        auto callback = [this](const Pose::UniquePtr msg) -> void
        {
            double x { msg->position.x };
            double y { msg->position.y };
            double z { msg->position.z };
            Quaternion q { msg->orientation };
            Robotics::Kinematics_6DOF_RobotArm::Quaternion quat { q.w, q.x, q.y, q.z };
            Robotics::Kinematics_6DOF_RobotArm::Pose target_pose { {x, y, z}, quat };

            auto solutions { RAK.solveAll(target_pose) };
            if ( solutions.empty() ) {
                RCLCPP_WARN(this->get_logger(), "No IK solution found for the given target pose.");
                return;
            }
            auto joint_angle_msg = interfaces::msg::JointAngleArray();
            joint_angle_msg.angles[0] = solutions[0].joint_angles[0];
            joint_angle_msg.angles[1] = solutions[0].joint_angles[1];
            joint_angle_msg.angles[2] = solutions[0].joint_angles[2];
            joint_angle_msg.angles[3] = solutions[0].joint_angles[3];
            joint_angle_msg.angles[4] = solutions[0].joint_angles[4];
            joint_angle_msg.angles[5] = solutions[0].joint_angles[5];

            RCLCPP_INFO(this->get_logger(), "IK solution found: [%f, %f, %f, %f, %f, %f]", solutions[0].joint_angles[0], solutions[0].joint_angles[1], solutions[0].joint_angles[2],                                                                           solutions[0].joint_angles[3], solutions[0].joint_angles[4], solutions[0].joint_angles[5]);
            joint_angle_publisher_->publish(joint_angle_msg);
        };
        // Subscriber
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("pose_topic", 10, callback);
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<interfaces::msg::JointAngleArray>::SharedPtr joint_angle_publisher_;

    Robotics::Kinematics_6DOF_RobotArm RAK;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
