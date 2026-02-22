#ifndef KINEMATICS_BASE_H
#define KINEMATICS_BASE_H

#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>
#include <optional>


namespace Robotics {

// Function to wrap an angle to the range [-pi, pi)
inline double wrapToPi(double angle) {
    // In C++20 and later, use std::numbers::pi
    // For older C++, you can use M_PI from <cmath> (if available) or calculate it (e.g., 4 * atan(1.0)).
    // Here we use M_PI for broad compatibility, ensure your compiler defines it (often requires a specific macro like _USE_MATH_DEFINES on Windows).
    
    double pi = std::acos(-1.0); // A reliable way to get PI if M_PI is unavailable
    double two_pi = 2.0 * pi;

    // Use fmod to constrain the angle to a range of width 2*pi,
    // but the result range is based on the sign of angle (e.g. [-2pi, 2pi) or [0, 2pi)).
    // A correction is needed for negative results to fit into [-pi, pi).

    double wrapped = std::fmod(angle + pi, two_pi);
    if (wrapped < 0) {
        wrapped += two_pi;
    }
    // Now 'wrapped' is in the range [0, 2*pi). Shift it to [-pi, pi).
    return wrapped - pi;
}

template <typename T, std::size_t DOF>
class KinematicsBase {
public:
    using Scalar = T;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Quaternion = Eigen::Quaternion<T>;
    using Matrix4 = Eigen::Matrix<T, 4, 4>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;


    struct JointConstraint {
        T min_angle {-M_PI};
        T max_angle {M_PI};
        T max_velocity {};
        T max_acceleration {};
        T max_jerk {};

        bool isWithinLimits(T angle) const {
            bool within_limits { angle >= min_angle && angle <= max_angle};
            // return true; // temporary: ignore limits
            return within_limits;
        }
        T clamp(T angle) const {
            return std::clamp(angle, min_angle, max_angle);
        }
    };

    using JointAngles = std::array<T, DOF>;
    using JointConstraints = std::array<JointConstraint, DOF>;

    /**
     * \brief Represents the pose of the end-effector, including position and orientation.
     * The position is represented as a 3D vector, and the orientation is represented as a quaternion.
     * @param position (Vector3) The position of the end-effector in 3D space.
     * @param orientation (Quaternion) The orientation of the end-effector, represented as a quaternion.
     */
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
    virtual ~KinematicsBase() = default;

public:
    // return the first solution found for the target pose
    virtual IKSolution solve(const Vector3& target_position) = 0;
    virtual IKSolution solve(const Pose& targetPose) = 0;
    // return the solution closest to the current joint angles
    // virtual IKSolution solve(const Matrix4& targetPose, const JointAngles& current_angles);

    virtual std::vector<IKSolution> solveAll(const Pose& target_pose) = 0;
    virtual std::vector<IKSolution> solveAll(const Pose& target_pose, const JointAngles& inital_cond) = 0;


    // virtual std::optional<Pose> FK(const JointAngles& joint_angles) = 0;
    virtual std::optional<Matrix4> FK(const JointAngles& joint_angles) = 0;
    
    virtual bool isReachable(const Pose& target_pose) const = 0;

protected:
    Matrix4 DH_trans(const T alpha, const T a, const T d, const T theta) {
        return Matrix4 {
            {cos(theta), -sin(theta), 0, a},
            {sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d},
            {sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d},
            {0, 0, 0, 1}
        };
    }
    
protected:
    JointConstraints m_joint_constraints {};


};

} // namespace robotics

#endif // KINEMATICS_BASE_H