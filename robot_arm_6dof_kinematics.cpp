#include "robot_arm_6dof_kinematics.h"
#include <algorithm>

namespace Robotics {

std::array<double, 6> operator-(const std::array<double, 6>& arr1, const std::array<double, 6>& arr2) {
    std::array<double, 6> ans;
    for ( std::size_t i {}; i < 6; ++i)
        ans[i] = arr1[i] - arr2[i];
    return ans;
}

std::ostream& operator<<(std::ostream& out, std::array<double, 6>& arr) {
    for ( std::size_t i {0}; i < 6; ++i ){
        out << arr[i] << ' ';
    }
    return out;
}


// Kinematics_6DOF_RobotArm::Kinematics_6DOF_RobotArm(const JointConstraints& jc) : m_joint_constraints{ jc } {}

Kinematics_6DOF_RobotArm::IKSolution
Kinematics_6DOF_RobotArm::solve(const Pose& target_pose) {
    // TODO: add a criterion to determine which solution to return
    // for now just the first solution is returned
    std::vector<IKSolution> allSolutions { solveAll(target_pose) };
    if ( !allSolutions.empty() )
        return allSolutions[0];
    return {};
}
Kinematics_6DOF_RobotArm::IKSolution
Kinematics_6DOF_RobotArm::solve(const Vector3& position) {
    return {};
}

std::vector<Kinematics_6DOF_RobotArm::IKSolution>
Kinematics_6DOF_RobotArm::solveAll(const Pose& target_pose, const JointAngles& initial_cond) {
    std::vector<IKSolution> allSolutions { solveAll(target_pose) };
    // TODO: not the best way to return empty solution
    if ( allSolutions.empty() )
        return {};
    
    std::sort(
        allSolutions.begin(),
        allSolutions.end(),
        [initial_cond](IKSolution& sA, IKSolution& sB ) {
            return arrayAbsSum<double, 6>(sA.joint_angles - initial_cond) < arrayAbsSum<double, 6>(sB.joint_angles - initial_cond);
        }
    );
    return allSolutions;
}

std::vector<Kinematics_6DOF_RobotArm::IKSolution>
Kinematics_6DOF_RobotArm::solveAll(const Pose& target_pose) {
    const Matrix4 transMat { target_pose.toMatrix() };
    
    // solve t1
    std::vector<PartialSolution> stage1;
    solveT1(stage1, transMat);

    // use stage1 to solve t3
    std::vector<PartialSolution> stage3;
    for ( const PartialSolution& ps : stage1 ) {
        solveT3(stage3, transMat, ps);
    }

    // use stage3 to solve t2
    std::vector<PartialSolution> stage2;
    for ( const PartialSolution& ps : stage3 ) {
        solveT2(stage2, transMat, ps);
    }
    
    // use stage2 to solve t4
    std::vector<PartialSolution> stage4;
    for ( const PartialSolution& ps : stage2 ) {
        solveT4(stage4, transMat, ps);
    }

    std::vector<PartialSolution> stage5;
    for ( const PartialSolution& ps : stage4 ) {
        solveT5(stage5, transMat, ps);
    }

    std::vector<PartialSolution> stage6;
    for ( const PartialSolution& ps : stage5 ) {
        solveT6(stage6, transMat, ps);
    }

    // TODO: add metadata validation
    // temporary: no metadata validation
    std::vector<IKSolution> solution;
    for ( const PartialSolution& ps : stage6 ) {
        solution.push_back( IKSolution{ ps, {}, {}});
    }
    return solution;
}


// TODO

std::optional<Kinematics_6DOF_RobotArm::Pose>
Kinematics_6DOF_RobotArm::FK(const JointAngles& joint_angles) const {
    return {};
}

bool Kinematics_6DOF_RobotArm::isReachable(const Pose& target_pose) const {
     return false;
}


void
Kinematics_6DOF_RobotArm::solveT1(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat) {
    const double px { transMat(0, 3) };
    const double py { transMat(1, 3) };
    double t1 {};

    // case 1
    // theta1 = atan2(py, px)
    t1 = atan2(py, px);
    if ( m_joint_constraints[0].isWithinLimits(t1) )      // check range
        potential_solutions.push_back( PartialSolution {t1, 0, 0, 0, 0, 0} );
        // potential_solutions.emplace_back( t1, 0, 0, 0, 0, 0 );

    // case 2
    // theta1 = atan2(-py, -px)
    t1 = atan2(-py, -px);
    if ( m_joint_constraints[0].isWithinLimits(t1) )      // check range
        potential_solutions.push_back( PartialSolution {t1, 0, 0, 0, 0, 0} );
        // potential_solutions.emplace_back( t1, 0, 0, 0, 0, 0 );
}

void
Kinematics_6DOF_RobotArm::solveT3(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps) {
    const double px { transMat(0, 3) };
    const double py { transMat(1, 3) };
    const double pz { transMat(2, 3) };
    const double t1 { ps[0] };

    double s3 { 1. / m_a2 / m_d4 };
    s3 *= ( pow((px * cos(t1) + py * sin(t1)), 2) + pow((pz - m_d1), 2) - pow(m_a2, 2) - pow(m_d4, 2));

    // no solution. shouldn't happen though
    if ( abs( s3 ) > 1)
        return;

    double c3 { sqrt( 1 - pow(s3, 2) ) };
    double t3 {};
    
    // case 1
    t3 = atan2(s3, c3);
    if ( m_joint_constraints[2].isWithinLimits(t3) )      // check range
        potential_solutions.push_back( PartialSolution{ t1, 0, t3, 0, 0, 0 } );
    // case 2
    t3 = atan2(s3, -c3);
    if ( m_joint_constraints[2].isWithinLimits(t3) )      // check range
        potential_solutions.push_back( PartialSolution{ t1, 0, t3, 0, 0, 0 } );
}

void
Kinematics_6DOF_RobotArm::solveT2(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps) {
    const double px { transMat(0, 3) };
    const double py { transMat(1, 3) };
    const double pz { transMat(2, 3) };
    const double t1 { ps[0] };
    const double t3 { ps[2] };

    // target t2

    // coefficients
    const double A2 { px * cos(t1) + py * sin(t1) };
    const double B2 { pz - m_d1 };
    const double D2 { m_a2 + m_d4 * sin(t3) };
    const double E2 { pz - m_d1 };
    const double F2 { -A2 };
    const double G2 { -m_d4 * cos(t3) };
    const double denominator { A2 * F2 - B2 * E2 };
    
    const double c2 { ( D2 * F2 - B2 * G2 ) / denominator };
    const double s2 { ( A2 * G2 - E2 * D2 ) / denominator };
    
    // pruning. no valid result
    if ( abs( c2 ) > 1 || abs( s2 ) > 1 )
        return;
    double t2 { atan2(s2, c2) };
    if ( m_joint_constraints[1].isWithinLimits(t2) )      // check range
        potential_solutions.push_back( PartialSolution{ t1, t2, t3, 0, 0, 0 });
}

void
Kinematics_6DOF_RobotArm::solveT4(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps) {
    const double px { transMat(0, 3) };
    const double py { transMat(1, 3) };
    const double pz { transMat(2, 3) };
    const double r13 { transMat(0, 2) };
    const double r23 { transMat(1, 2) };
    const double r33 { transMat(2, 2) };

    const double t1 { ps[0] };
    const double t2 { ps[1] };
    const double t3 { ps[3] };
    const double t23 { t2+t3 };

    // s4 -> s4/s5
    // c4 -> c4/s5
    double s4 { r13 * sin(t1) - r23 * cos(t1) };
    double c4 { r33 * sin(t23) + r23 * sin(t1) * cos(t23) + r13 * cos(t1) * cos(t23) };

    // pruning. no valid result
    if ( abs(s4) > 1 || abs(c4) > 1)
        return;
    
    double t4 {};
    // case 1
    t4 = atan2(s4, c4);
    if ( m_joint_constraints[3].isWithinLimits(t4) )      // check range
        potential_solutions.push_back( PartialSolution{t1, t2, t3, t4, 0, 0} );
        // potential_solutions.emplace_back( t1, t2, t3, t4, 0, 0 );
    t4 = atan2(-s4, -c4);
    if ( m_joint_constraints[3].isWithinLimits(t4) )      // check range
        potential_solutions.push_back( PartialSolution{t1, t2, t3, t4, 0, 0} );
        // potential_solutions.emplace_back( t1, t2, t3, t4, 0, 0 );


}
void
Kinematics_6DOF_RobotArm::solveT5(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps) {
    const double r13 { transMat(0, 2) };
    const double r23 { transMat(1, 2) };
    const double r33 { transMat(2, 2) };

    const double t1 { ps[0] };
    const double t2 { ps[1] };
    const double t3 { ps[2] };
    const double t4 { ps[3] };
    const double t23 { t2+t3 };

    double s5 {};
    double c5 {};
    s5 = r13 * ( sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t23) ) -
         r23 * ( cos(t1)*sin(t4) - sin(t1)*cos(t4)*cos(t23) ) +
         r33 * sin(t4)*sin(t23);
    if ( abs(s5) > 1 )  return;
    c5 = r13 * cos(t1)*sin(t23) + r23 * sin(t1)*sin(t23) - r33 * cos(t23);
    if ( abs(c5) > 1)  return;
    double t5 { atan2(s5, c5) };
    if ( m_joint_constraints[4].isWithinLimits(t5) )      // check range
        potential_solutions.push_back(PartialSolution{t1, t2, t3, t4, t5, 0});
        // potential_solutions.emplace_back(t1, t2, t3, t4, t5, 0);
}

void
Kinematics_6DOF_RobotArm::solveT6(std::vector<PartialSolution>& potential_solutions, const Matrix4& transMat, const PartialSolution& ps) {
    const double r11 { transMat(0, 0) };
    const double r21 { transMat(1, 0) };
    const double r31 { transMat(2, 0) };

    const double t1 { ps[0] };
    const double t2 { ps[1] };
    const double t3 { ps[2] };
    const double t4 { ps[3] };
    const double t5 { ps[4] };
    const double t23 { t2+t3 };

    double s6 {};
    double c6 {};
    s6 = r11 * (cos(t4)*sin(t1) - sin(t4)*cos(t1)*cos(t23))
        -r21 * (cos(t1)*cos(t4) + sin(t4)*sin(t1)*sin(t23))
        -r31 * ( sin(t4)*sin(t23) );
    if ( abs(s6) > 1 )  return;
    c6 = r11 * (sin(t1)*sin(t4)*cos(t5) + cos(t1)*cos(t4)*cos(t5)*cos(t23) - cos(t1)*sin(t5)*sin(t23))
        -r21 * (cos(t1)*sin(t4)*cos(t5) - sin(t1)*cos(t4)*sin(t5)*cos(t23) + sin(t1)*sin(t5)*sin(t23))
        +r31 * (sin(t5)*cos(t23) + cos(t4)*cos(t5)*sin(t23));
    if ( abs(c6) > 1 )  return;

    double t6 { atan2(s6, c6) };
    if ( m_joint_constraints[5].isWithinLimits(t6) )      // check range
        potential_solutions.push_back(PartialSolution{t1, t2, t3, t4, t5, t6});

}
 

}       // Robotics
