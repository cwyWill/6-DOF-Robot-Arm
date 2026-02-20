#ifndef ROBOT_ARM_6DOF_KINEMATICS_H
#define ROBOT_ARM_6DOF_KINEMATICS_H

#define _USE_MATH_DEFINES
// pi: M_PI

#include <cmath>
#include "src/kinematics_base.h"
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
    // Kinematics_6DOF_RobotArm( const JointConstraints& jc);
    void setConstraints(const JointConstraints& jc);

    // IKSolution solve(const Vector3& target_position) = 0;
    IKSolution solve(const Pose& target_pose) override;
    IKSolution solve(const Vector3& target_position) override;

    //
    std::vector<IKSolution> solveAll(const Pose& target_pose) override;

    std::vector<IKSolution> solveAll(const Pose& target_pose, const JointAngles& inital_cond) override;

    // std::optional<Pose> FK(const JointAngles& joint_angles) const;
    std::optional<Matrix4> FK(const JointAngles& joint_angles);
    bool isReachable(const Pose& target_pose) const override;

    
public:
    void updateEndEffectorLength(double length ) {
        m_endeffector_length = length;
        m_d6 = m_d6_offset + m_endeffector_length;
    }


private:
    constexpr static double m_alpha1 { -M_PI_2 };
    constexpr static double m_alpha3 { -M_PI_2 };
    constexpr static double m_alpha4 {  M_PI_2 };
    constexpr static double m_alpha5 { -M_PI_2 };

    constexpr static double m_d1 { 60 };
    constexpr static double m_d4 { 135 };
    constexpr static double m_d6_offset { 65 };
    double m_endeffector_length { 25 };
    double m_d6 { m_d6_offset + m_endeffector_length };

    constexpr static double m_a2 { 110 };

private:
    using PartialSolution = JointAngles;
    void solveT1(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat);
    // potential_solution: out parameter
    void solveT3(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);
    // potential_solution: out parameter
    void solveT2(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);
    // potential_solution: out parameter
    void solveT45(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps, const JointAngles& initial_cond);
    // potential_solution: out parameter
    void solveT6(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps);

};

} // namespace Robotics

#endif