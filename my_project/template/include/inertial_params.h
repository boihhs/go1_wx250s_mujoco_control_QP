#ifndef INERTIAL_PARAMS_H
#define INERTIAL_PARAMS_H

#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <iostream>
#include <tuple>
#include "utils.h"
#include "globals.h"

// Function to calculate the inertial parameters (COM and MOI) of the robotic arm
// Input: qArmInput - A 6D vector of joint angles
// Output: A pair containing the center of mass (Eigen::Vector3d) and the moment of inertia (Eigen::Matrix3d)
std::pair<Eigen::Vector3d, Eigen::Matrix3d> inertial_params(const Eigen::VectorXd& qArmInput);

#endif // INERTIAL_PARAMS_H
