#include "utils.h"


// Helper functions for rotations
Eigen::Matrix3d quat2rotm(const Eigen::Vector4d& q) {
    Eigen::Quaterniond quaternion(q(0), q(1), q(2), q(3)); // Correct order: w, x, y, z
    return quaternion.toRotationMatrix();
}

Eigen::Vector3d normalizeAngles(const Eigen::Vector3d& angles) {
    Eigen::Vector3d normalized;
    for (int i = 0; i < 3; ++i) {
        normalized[i] = std::atan(std::sin(angles[i])/std::cos(angles[i]));
        std::cout << " \n angles: " << angles[i];
    }
    return normalized;
}

Eigen::Vector3d rotm2eul(const Eigen::Matrix3d& R) {
    if (std::abs(R(2, 0)) < 1.0) {
        double yaw = std::atan2(R(1, 0), R(0, 0)); // Rotation around Z-axis
        double pitch = std::asin(-R(2, 0));        // Rotation around Y-axis
        double roll = std::atan(R(2, 1)/ R(2, 2)); // Rotation around X-axis
        return Eigen::Vector3d(yaw, pitch, roll);
    } else {
        // Handle gimbal lock case
        double yaw = std::atan(-R(0, 1)/ R(1, 1));
        double pitch = (R(2, 0) > 0) ? -M_PI_2 : M_PI_2; // +/- 90 degrees
        double roll = 0; // Undefined, set to zero
        return Eigen::Vector3d(yaw, pitch, roll);
    }
}

Eigen::Vector3d rotm2eul2(const Eigen::Matrix3d& R) {
    return R.eulerAngles(2, 1, 0);
    
}

Eigen::Matrix3d eul2rotm(const Eigen::Vector3d& euler) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
    return R;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
    Eigen::Matrix3d b;
    b << 0, -a.z(), a.y(),
         a.z(), 0, -a.x(),
         -a.y(), a.x(), 0;
    return b;
}

Eigen::Matrix3d Rx(double a) {
    Eigen::Matrix3d M;
    M << 1, 0, 0,
         0, cos(a), -sin(a),
         0, sin(a), cos(a);
    return M;
}

Eigen::Matrix3d Ry(double a) {
    Eigen::Matrix3d M;
    M << cos(a), 0, sin(a),
         0, 1, 0,
         -sin(a), 0, cos(a);
    return M;
}

Eigen::Matrix3d Rz(double a) {
    Eigen::Matrix3d M;
    M << cos(a), -sin(a), 0,
         sin(a), cos(a), 0,
         0, 0, 1;
    return M;
}