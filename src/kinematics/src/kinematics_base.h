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
    double wrapped = std::fmod(angle + M_PI, M_PI*2.);
    if (wrapped < 0) {
        wrapped += M_PI*2.;
    }
    return wrapped - M_PI;
}

template <typename T, std::size_t DOF>
class KinematicsBase {
public:
    using Scalar = T;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Quaternion = Eigen::Quaternion<T>;
    using Matrix4 = Eigen::Matrix<T, 4, 4>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;

    /**
     * @brief The constraint of a joint, including limits on angle, velocity, acceleration, and jerk.
     */
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
        /**
         * @brief Clamp the angle to be within the joint limits.
         * 
         * @param angle 
         * @return clamped angle
         */
        T clamp(T angle) const {
            return std::clamp(angle, min_angle, max_angle);
        }
    };

    /**
     * @brief An array of joint angles
     * @tparam T The scalar type for joint angles.
     */
    using JointAngles = std::array<T, DOF>;
    /**
     * @brief An array of JointConstraint.
     */
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

    /**
     * @brief The metadata of an inverse kinematics solution
     */
    struct SolutionMetadata {
        T position_error {};
        T orientation_error {};
        std::size_t iterations {};
        bool convered { false };
        T condition_number {};
    };

    /**
     * @brief The inverse kinematis solution, including joint angles and solution metadata.
     */
    struct IKSolution {
        JointAngles joint_angles {};
        SolutionMetadata metadata {};
        bool is_valid {};
    };

public:
    /**
     * @brief Destroy the Kinematics Base object
     */
    virtual ~KinematicsBase() = default;

public:
    // /**
    //  * @brief Solve the inverse kinematics with a given target position.
    //  * @param target_position 
    //  * @return IKSolution 
    //  */
    // virtual IKSolution solve(const Vector3& target_position) = 0;

    /**
     * @brief Solve the inverse kinematics with a given target pose.
     * @param pose 
     * @return IKSolution The inverse kinematics solution.
     */
    virtual IKSolution solve(const Pose& targetPose) = 0;
    // return the solution closest to the current joint angles
    // virtual IKSolution solve(const Matrix4& targetPose, const JointAngles& current_angles);

    /**
     * @brief Solve all IK solution with a given target pose.
     * 
     * @param target_pose 
     * @return std::vector<IKSolution> 
     */
    virtual std::vector<IKSolution> solveAll(const Pose& target_pose) = 0;
    /**
     * @brief Solve all IK solution with a given target pose. If initial condition is provided, the solutions are sorted by joint angle errors.
     * 
     * @param target_pose 
     * @return std::vector<IKSolution> 
     */
    virtual std::vector<IKSolution> solveAll(const Pose& target_pose, const JointAngles& inital_cond) = 0;


    virtual Matrix4 FK(const JointAngles& joint_angles) = 0;
    
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