#ifndef utils_H
#define utils_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <cmath>

Eigen::Matrix3d quat2rotm(const Eigen::Vector4d& q);
Eigen::Vector3d normalizeAngles(const Eigen::Vector3d& angles);
Eigen::Vector3d rotm2eul(const Eigen::Matrix3d& R);
Eigen::Matrix3d eul2rotm(const Eigen::Vector3d& euler);
Eigen::Matrix3d skew(const Eigen::Vector3d& a);
Eigen::Matrix3d Rx(double a);
Eigen::Matrix3d Ry(double a);
Eigen::Matrix3d Rz(double a);

#endif