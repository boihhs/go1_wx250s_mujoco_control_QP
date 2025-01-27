#include "contact_jacobian.h"

Eigen::VectorXd ContactJacobian_01(const Eigen::VectorXd& xin, const Eigen::VectorXd& q, const Eigen::VectorXd& F_MPC) {

    
    Eigen::MatrixXd Jc1 = ContactJacobian(xin, q);
    Eigen::VectorXd GRF = -F_MPC;

    Eigen::VectorXd tau = Jc1.transpose() * GRF;

    return tau;

}