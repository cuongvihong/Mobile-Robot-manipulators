#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>

namespace myrobot_kinematics
{

using JointArray = std::array<double, 4>;

double wrapToPi(double a);

// FK đúng theo Matlab bạn đã gửi
Eigen::Matrix4d forwardKinematics(const JointArray& q);

// Dựng T05 từ x,y,z,phi
// x,y,z: mét
// phi: rad
Eigen::Matrix4d buildT05FromXYZPhi(double x, double y, double z, double phi);

// IK 2 nghiệm từ T05
bool solveIK2Solutions(const Eigen::Matrix4d& T05, JointArray& sol1, JointArray& sol2);

// Sai số pose để verify FK
double poseError(const Eigen::Matrix4d& T_target, const Eigen::Matrix4d& T_check);

// Kiểm tra giới hạn khớp
bool exceedsJointLimits(const JointArray& q, double eps = 1e-9);

// Chọn nghiệm tốt nhất
bool chooseBestSolution(
    const Eigen::Matrix4d& T_target,
    const JointArray& sol1,
    const JointArray& sol2,
    const std::vector<double>& current_joints,
    JointArray& best_sol);

}  // namespace myrobot_kinematics