#ifndef get_vel_feel_arm_H
#define get_vel_feel_arm_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "utils_jac.h"


Eigen::VectorXd legVelocity(const Eigen::VectorXd& xin, const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

Eigen::Vector3d armVelocity(const Eigen::VectorXd& xin, const Eigen::VectorXd& q_arm, const Eigen::VectorXd& dq_arm);


#endif