#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <vector>
#include <cmath>
#include "inertial_params.h"
#include "utils.h"
#include "globals.h"


Eigen::VectorXd CartesianControl_01(double t, const Eigen::VectorXd& xin, const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& xdes, const Eigen::VectorXd& fpfv,
                                    const Eigen::VectorXd& qArm);
// Helper functions for leg dynamics
std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeLegJacobianAndPosition(const Eigen::Vector3d& q, int leg);
Eigen::VectorXd LegTorque_quadruped(const std::vector<Eigen::Vector3d>& F, const Eigen::VectorXd& q, double t);



#endif