#include "myrobot_commander_cpp/ik_myrobot.hpp"

#include <cmath>
#include <algorithm>

namespace myrobot_kinematics
{

namespace
{
// ===== Robot parameters (mm -> m) =====
constexpr double d1 = 30.0e-3;
constexpr double d2 = -29.66e-3;
constexpr double d3 = 9.07e-3;

constexpr double l1 = 228.89e-3;
constexpr double l2 = 228.89e-3;
constexpr double l3 = 125.09e-3;
constexpr double l4 = 83.65e-3;

// Offset ngang phụ trong FK
constexpr double D = d1 + d2 + d3;   // = 0.00941 m

Eigen::Matrix4d MDH(double ai_1, double alpha_i_1, double d, double theta)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T(0, 0) = std::cos(theta);
    T(0, 1) = -std::sin(theta);
    T(0, 2) = 0.0;
    T(0, 3) = ai_1;

    T(1, 0) = std::cos(alpha_i_1) * std::sin(theta);
    T(1, 1) = std::cos(alpha_i_1) * std::cos(theta);
    T(1, 2) = -std::sin(alpha_i_1);
    T(1, 3) = -d * std::sin(alpha_i_1);

    T(2, 0) = std::sin(alpha_i_1) * std::sin(theta);
    T(2, 1) = std::sin(alpha_i_1) * std::cos(theta);
    T(2, 2) = std::cos(alpha_i_1);
    T(2, 3) = d * std::cos(alpha_i_1);

    return T;
}

double jointDistanceCost(const JointArray& q, const std::vector<double>& current_joints)
{
    if (current_joints.size() < 4) {
        return 0.0;
    }

    double cost = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const double dq = wrapToPi(q[i] - current_joints[i]);
        cost += dq * dq;
    }
    return cost;
}

}  // namespace

double wrapToPi(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

Eigen::Matrix4d forwardKinematics(const JointArray& q)
{
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];
    const double q4 = q[3];

    const Eigen::Matrix4d T1 = MDH(0.0, 0.0, l3, q1);
    const Eigen::Matrix4d T2 = MDH(0.0, -M_PI / 2.0, d1, q2 - M_PI / 2.0);
    const Eigen::Matrix4d T3 = MDH(l1, 0.0, d2, q3);
    const Eigen::Matrix4d T4 = MDH(l2, 0.0, 0.0, q4);
    const Eigen::Matrix4d T5 = MDH(l4, 0.0, d3, 0.0);

    return T1 * T2 * T3 * T4 * T5;
}

Eigen::Matrix4d buildT05FromXYZPhi(double x, double y, double z, double phi)
{
    // Từ FK:
    // Px = cos(q1)*A - sin(q1)*D
    // Py = sin(q1)*A + cos(q1)*D
    // => A = sqrt(x^2 + y^2 - D^2)
    // => q1 = atan2(y, x) - atan2(D, A)

    const double r2 = x * x + y * y;
    const double A_sq = r2 - D * D;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    if (A_sq < 0.0) {
        return T;
    }

    const double A = std::sqrt(A_sq);
    const double q1 = std::atan2(y, x) - std::atan2(D, A);

    const double s1 = std::sin(q1);
    const double c1 = std::cos(q1);
    const double sphi = std::sin(phi);
    const double cphi = std::cos(phi);

    // Rotation rút từ FK
    T(0, 0) = sphi * c1;
    T(0, 1) = cphi * c1;
    T(0, 2) = -s1;

    T(1, 0) = s1 * sphi;
    T(1, 1) = s1 * cphi;
    T(1, 2) = c1;

    T(2, 0) = cphi;
    T(2, 1) = -sphi;
    T(2, 2) = 0.0;

    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;

    return T;
}

bool solveIK2Solutions(const Eigen::Matrix4d& T05, JointArray& sol1, JointArray& sol2)
{
    const Eigen::Matrix3d R = T05.block<3, 3>(0, 0);
    const double Px = T05(0, 3);
    const double Py = T05(1, 3);
    const double Pz = T05(2, 3);

    // 1) q1 từ rotation
    const double q1 = std::atan2(-R(0, 2), R(1, 2));
    const double c1 = std::cos(q1);
    const double s1 = std::sin(q1);

    // 2) theta234 từ rotation
    const double theta234 = std::atan2(-R(2, 1), R(2, 0));

    // 3) Từ FK:
    // A = c1*Px + s1*Py - D
    const double A = c1 * Px + s1 * Py - D;
    const double Z = Pz - l3;

    const double Aw = A - l4 * std::sin(theta234);
    const double Zw = Z - l4 * std::cos(theta234);

    const double r2 = Aw * Aw + Zw * Zw;

    // 4) Solve q3
    double c3 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

    constexpr double tol = 1e-9;
    if (c3 > 1.0 + tol || c3 < -1.0 - tol) {
        return false;
    }

    c3 = std::max(-1.0, std::min(1.0, c3));
    const double s3_abs = std::sqrt(std::max(0.0, 1.0 - c3 * c3));

    // Branch 1
    {
        const double q3 = std::atan2(+s3_abs, c3);
        const double q2 = std::atan2(Aw, Zw) - std::atan2(l2 * std::sin(q3), l1 + l2 * std::cos(q3));
        const double q4 = theta234 - q2 - q3;

        sol1 = {
            wrapToPi(q1),
            wrapToPi(q2),
            wrapToPi(q3),
            wrapToPi(q4)
        };
    }

    // Branch 2
    {
        const double q3 = std::atan2(-s3_abs, c3);
        const double q2 = std::atan2(Aw, Zw) - std::atan2(l2 * std::sin(q3), l1 + l2 * std::cos(q3));
        const double q4 = theta234 - q2 - q3;

        sol2 = {
            wrapToPi(q1),
            wrapToPi(q2),
            wrapToPi(q3),
            wrapToPi(q4)
        };
    }

    return true;
}

double poseError(const Eigen::Matrix4d& T_target, const Eigen::Matrix4d& T_check)
{
    const Eigen::Vector3d p_target = T_target.block<3, 1>(0, 3);
    const Eigen::Vector3d p_check = T_check.block<3, 1>(0, 3);

    const double pos_err = (p_target - p_check).norm();

    const Eigen::Matrix3d R_target = T_target.block<3, 3>(0, 0);
    const Eigen::Matrix3d R_check = T_check.block<3, 3>(0, 0);

    Eigen::Matrix3d R_err = R_target.transpose() * R_check;
    double trace_val = (R_err.trace() - 1.0) * 0.5;
    trace_val = std::max(-1.0, std::min(1.0, trace_val));
    const double rot_err = std::acos(trace_val);

    return pos_err + 0.1 * rot_err;
}

bool exceedsJointLimits(const JointArray& q, double eps)
{
    // Chỉnh các giới hạn này đúng theo URDF nếu cần.
    const double q1_min = -M_PI, q1_max =  M_PI;
    const double q2_min = -M_PI, q2_max =  M_PI;
    const double q3_min = -M_PI, q3_max =  M_PI;
    const double q4_min = -2.7,  q4_max =  2.7;

    return (q[0] < q1_min - eps || q[0] > q1_max + eps) ||
           (q[1] < q2_min - eps || q[1] > q2_max + eps) ||
           (q[2] < q3_min - eps || q[2] > q3_max + eps) ||
           (q[3] < q4_min - eps || q[3] > q4_max + eps);
}

bool chooseBestSolution(
    const Eigen::Matrix4d& T_target,
    const JointArray& sol1,
    const JointArray& sol2,
    const std::vector<double>& current_joints,
    JointArray& best_sol)
{
    const bool valid1 = !exceedsJointLimits(sol1);
    const bool valid2 = !exceedsJointLimits(sol2);

    if (!valid1 && !valid2) {
        return false;
    }
    if (valid1 && !valid2) {
        best_sol = sol1;
        return true;
    }
    if (!valid1 && valid2) {
        best_sol = sol2;
        return true;
    }

    const Eigen::Matrix4d T1 = forwardKinematics(sol1);
    const Eigen::Matrix4d T2 = forwardKinematics(sol2);

    const double err1 = poseError(T_target, T1);
    const double err2 = poseError(T_target, T2);

    const double cost1 = jointDistanceCost(sol1, current_joints);
    const double cost2 = jointDistanceCost(sol2, current_joints);

    constexpr double err_eps = 1e-6;

    if (std::abs(err1 - err2) > err_eps) {
        best_sol = (err1 < err2) ? sol1 : sol2;
        return true;
    }

    best_sol = (cost1 <= cost2) ? sol1 : sol2;
    return true;
}

}  // namespace myrobot_kinematics