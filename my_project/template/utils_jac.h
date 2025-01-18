#ifndef utils_jac_H
#define utils_jac_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "utils.h"

Eigen::MatrixXd ContactJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& q);

Eigen::MatrixXd ContactJacobianArm(const Eigen::VectorXd& x, const Eigen::VectorXd& q_arm);

#endif