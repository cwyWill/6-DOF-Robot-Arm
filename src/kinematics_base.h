#ifndef KINEMATICS_BASE_H
#define KINEMATICS_BASE_H

#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>
#include <optional>


namespace Robotics {



template <typename T, std::size_t DOF>
class KinematicsBase {
public:
    using Scalar = T;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Quaternion = Eigen::Quaternion<T>;
    using Matrix4 = Eigen::Matrix<T, 4, 4>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;


    struct JointConstraint {
        T min_angle {-180};
        T max_angle {180};
        T max_velocity {};
        T max_acceleration {};
        T max_jerk {};

        bool isWithinLimits(T angle) const {
            return angle >= min_angle && angle <= max_angle;
        }
        T clamp(T angle) const {
            return std::clamp(angle, min_angle, max_angle);
        }
    };

    using JointAngles = std::array<T, DOF>;
    using JointConstraints = std::array<JointConstraint, DOF>;

    struct Pose {
        Vector3 position { Vector3::Zero() };
        Quaternion orientation { Quaternion::Identity() };

        Matrix4 toMatrix() const {
            Matrix4 transMat { Matrix4::Identity() };
            transMat.template block<3, 3>(0, 0) = orientation.toRotationMatrix();
            transMat.template block<3, 1>(0, 3) = position;
            return transMat;
        }
    };

    struct SolutionMetadata {
        T position_error {};
        T orientation_error {};
        std::size_t iterations {};
        bool convered { false };
        T condition_number {};
    };

    struct IKSolution {
        JointAngles joint_angles {};
        SolutionMetadata metadata {};
        bool is_valid {};
    };

public:
    // KinematicsBase(const JointConstraints& joint_constraints) : m_joint_constraints { joint_constraints} {}
    ~KinematicsBase() = default;

public:
    // return the first solution found for the target pose
    virtual IKSolution solve(const Vector3& target_position) = 0;
    virtual IKSolution solve(const Pose& targetPose);
    // return the solution closest to the current joint angles
    // virtual IKSolution solve(const Matrix4& targetPose, const JointAngles& current_angles);

    virtual std::vector<IKSolution> solveAll(const Pose& target_pose) = 0;
    virtual std::vector<IKSolution> solveAll(const Pose& target_pose, const JointAngles& inital_cond) = 0;


    virtual std::optional<Pose> FK(const JointAngles& joint_angles) const = 0;
    
    virtual bool isReachable(const Pose& target_pose) const = 0;
    
protected:
    JointConstraints m_joint_constraints {};


};

} // namespace robotics

#endif // KINEMATICS_BASE_H