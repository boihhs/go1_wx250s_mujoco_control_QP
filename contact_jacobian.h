#ifndef CONTACT_JACOBIAN_H
#define CONTACT_JACOBIAN_H

#include <vector>
#include <Eigen/Dense> // For Matrix operations
#include "utils.h"
#include "utils_jac.h"
#include <cmath>


// Function declaration
Eigen::VectorXd ContactJacobian_01(const Eigen::VectorXd& x, const Eigen::VectorXd& q, const Eigen::VectorXd& F_MPC);

#endif