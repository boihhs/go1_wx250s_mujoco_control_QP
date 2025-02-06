#ifndef TRAJ_COMMAND_01M_H
#define TRAJ_COMMAND_01M_H
#include <iostream>
#include <Eigen/Dense>
#include "globals.h"

using namespace Eigen;

// Function declaration
VectorXd Traj_Command_01M(const double t, const VectorXd& xin);

#endif // TRAJ_COMMAND_01M_H
