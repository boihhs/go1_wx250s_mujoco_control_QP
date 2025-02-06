#ifndef IK_PD_End_H
#define IK_PD_End_H

#include <tuple>
#include <Eigen/Dense>
#include "utils.h"
#include "Traj_Command_01M.h"
#include "Traj_Command_Goal.h"

using namespace Eigen;

VectorXd IK_PD_End(const VectorXd& xin, const double t, const VectorXd& qArm, const Vector3d& goal);

#endif