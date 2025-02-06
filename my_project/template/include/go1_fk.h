#ifndef FK_H
#define FK_H

#include <tuple>
#include <Eigen/Dense>
#include "utils.h"

using namespace Eigen;

// VectorXd FK(const VectorXd& xin, const VectorXd& q, const VectorXd& qArm, const Eigen::VectorXd& dq, const Eigen::VectorXd& dq_arm);
VectorXd FK(const VectorXd& xin, const VectorXd& q, const VectorXd& qArm);

Matrix4d transformationMatrices(const Matrix3d rotation, const double one, const double two, const double three);

#endif