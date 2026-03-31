#ifndef ROBOT_ARM_6DOF_KINEMATICS_H
#define ROBOT_ARM_6DOF_KINEMATICS_H

#define _USE_MATH_DEFINES
// pi: M_PI

#include <cmath>
#include "kinematics_base.h"
#include <ostream>

namespace Robotics {

std::array<double, 6> operator-(const std::array<double, 6>& arr1, const std::array<double, 6>& arr2);

template <typename T, std::size_t N> 
T arrayAbsSum(std::array<T, N>&& arr1) {
    T result {};
    for ( const T& element : arr1) {
        result += abs(element);
    }
    return result;
}

std::ostream& operator<<(std::ostream& out, std::array<double, 6>& arr);

class Kinematics_6DOF_RobotArm : KinematicsBase<double, 6> {
public:
    using Base = KinematicsBase<double, 6>;
    using typename Base::Vector3;
    using typename Base::Matrix3;
    using typename Base::Matrix4;
    using typename Base::Quaternion;
    using typename Base::Pose;
    using typename Base::JointAngles;
    using typename Base::IKSolution;
    using typename Base::JointConstraints;
    using typename Base::SolutionMetadata;

public:
    /**
     * @brief Set the Constraints object
     * 
     * @param jc The joint constraints to be set.
     */
    void setConstraints(const JointConstraints& jc);

    /**
     * @brief Solve the inverse kinematics with a given target pose.
     * 
     * @param target_pose 
     * @return IKSolution 
     */
    IKSolution solve(const Pose& target_pose) override;

    /**
     * @brief Solve the inverse kinematics with a given target pose. If multiple solutions are available, the solution closest to the initial condition will be returned.
     * 
     * @param target_pose 
     * @return IKSolution 
     */
    IKSolution solve(const Pose& target_pose, const JointAngles& initial_cond);

    // TODO: solve solution with only given position
    // IKSolution solve(const Vector3& target_position) override;

    /**
     * @brief Solve all IK solution with a given target pose.
     * 
     * @param target_pose 
     * @return std::vector<IKSolution> 
     */
    std::vector<IKSolution> solveAll(const Pose& target_pose) override;

    /**
     * @brief Solve all IK solution with a given target pose. If initial condition is provided, the solutions are sorted by joint angle errors.
     * 
     * @param target_pose 
     * @return std::vector<IKSolution> 
     */
    std::vector<IKSolution> solveAll(const Pose& target_pose, const JointAngles& inital_cond) override;

    /**
     * @brief Return the transformation matrix of the 6-th frame.
     * 
     * @param joint_angles 
     * @return Matrix4
     */
    Matrix4 FK(const JointAngles& joint_angles);
    bool isReachable(const Pose& target_pose) const override;

    
public:
    /**
     * @brief Update the length of the end-effector. Used when tool is changed.
     * 
     * @param length 
     */
    void updateEndEffectorLength(double length ) {
        m_endeffector_length = length;
        m_d7 = m_d6_offset + m_endeffector_length;
    }

public:
    static constexpr JointAngles nominal_joint_angles { JointAngles{ 0, -2.3, 0, 0, 0, 0 } };   /** The nominal joint angles for the robot arm. Useful for initial state/condition setting. */

private:
    constexpr static double m_alpha1 { -M_PI_2 };
    constexpr static double m_alpha3 { -M_PI_2 };
    constexpr static double m_alpha4 {  M_PI_2 };
    constexpr static double m_alpha5 { -M_PI_2 };

    constexpr static double m_d1 { 0 };
    constexpr static double m_d4 { 135 };
    constexpr static double m_d6_offset { 65 };
    double m_endeffector_length { 25 }; /** The length of the end-effector */
    constexpr static double m_d6 { 0 };
    double m_d7 { m_d6 + m_endeffector_length }; /** The total length from the wrist center to the end-effector */


    constexpr static double m_a2 { 110 };

private:
    using PartialSolution = JointAngles;
    /**
     * @brief solve \theta_1.
     * 
     * This function solves the \theta_1 angle and validates if the solution is within the joint limits.
     * 
     * @param potential_solutions The out parameter to store potential solutions for \theta_1.
     * @param transMat The target transformation matrix.
     */
    void solveT1(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat);
    /**
     * @brief solve \theta_3
     * 
     * This function solves the \theta_3 angle and validates if the solution is within the joint limits.
     * 
     * @param potential_solutions The out parameter to store potential solutions for \theta_1 and \theta_3.
     * @param transMat The target transformation matrix.
     * @param ps The partial solution that is used to compute \theta_3.
     */
    void solveT3(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);
    /**
     * @brief solve \theta_2
     * 
     * This function solves the \theta_2 angle and validates if the solution is within the joint limits.
     * 
     * @param potential_solutions The out parameter to store potential solutions for \theta_1 through \theta_2.
     * @param transMat The target transformation matrix.
     * @param ps The partial solution that is used to compute \theta_2.
     */
    void solveT2(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);
    /**
     * @brief solve \theta_4 and \theta_5
     * 
     * This function solves the \theta_4 and \theta_5 angles and validates if the solution is within the joint limits.
     * 
     * @param potential_solutions The out parameter to store potential solutions for \theta_1 throught \theta_5.
     * @param transMat The target transformation matrix.
     * @param ps The partial solution that is used to compute \theta_4 and \theta_5.
     */
    void solveT45(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps, const JointAngles& initial_cond);
    /**
     * @brief solve \theta_6
     * 
     * This function solves the \theta_6 and validates if the solution is within the joint limits.
     * 
     * @param potential_solutions The out parameter to store potential solutions for \theta_1 throught \theta_6.
     * @param transMat The target transformation matrix.
     * @param ps The partial solution that is used to compute \theta_6.
     */
    void solveT6(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);

};

} // namespace Robotics

#endif