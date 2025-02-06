#ifndef TRAJ_COMMAND_Goal_H
#define TRAJ_COMMAND_Goal_H
#include <iostream>
#include <Eigen/Dense>
#include "globals.h"
#include <algorithm>

using namespace Eigen;

// Function declaration
VectorXd Traj_Command_Goal(const double t, const VectorXd& xin, const Vector3d& goal);

#endif 
