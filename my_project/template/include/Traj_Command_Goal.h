#ifndef TRAJ_COMMAND_Goal_H
#define TRAJ_COMMAND_Goal_H
#include <iostream>
#include <Eigen/Dense>
#include "globals.h"
#include <algorithm>
#include <cmath>

using namespace Eigen;

// Function declaration
VectorXd Traj_Command_Goal(const double t, const VectorXd& xin, const Vector3d& goal, const Vector3d& prev);
VectorXd Traj_Command_Goal2(const double t, const VectorXd& xin, const Vector3d& goal, const Vector3d& prev);

#endif 
