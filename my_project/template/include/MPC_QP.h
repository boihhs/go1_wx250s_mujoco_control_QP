#ifndef MPC_QP_H
#define MPC_QP_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "inertial_params.h" // Assuming this header exists
#include "Traj_Command_01M.h" // Assuming this header exists
#include "utils.h"
#include "globals.h"
#include "Traj_Command_Goal.h"

// Global Variables (consider using a class for better organization)
extern double dt_MPC;
extern double m_Object;

// Function Prototype for MPC_QP
Eigen::VectorXd MPC_QP(const Eigen::VectorXd& xin, const Eigen::VectorXd& PosVel, const Eigen::VectorXd& qArm, const Eigen::Vector3d& F_Tatile, double t, const Vector3d& goal, const Vector3d& prev);

#endif