#ifndef CONTACT_JACOBIAN_ARM_H
#define CONTACT_JACOBIAN_ARM_H

#include <vector>
#include <Eigen/Dense> // For Matrix operations
#include "utils.h"
#include <cmath>
#include "utils_jac.h"


// Function declaration
Eigen::VectorXd ContactJacobian_Arm(const Eigen::VectorXd& xin, const Eigen::VectorXd& q_arm, const Eigen::VectorXd& dq_arm, const Eigen::VectorXd& F_arm);

#endif