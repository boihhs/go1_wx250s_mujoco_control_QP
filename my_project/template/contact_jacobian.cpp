#include "contact_jacobian.h"

Eigen::VectorXd ContactJacobian_01(const Eigen::VectorXd& x, const Eigen::VectorXd& q, const Eigen::VectorXd& F_MPC) {

    
    Eigen::MatrixXd Jc1 = ContactJacobian(x, q);
    Eigen::VectorXd GRF = -F_MPC;

    Eigen::VectorXd tau = Jc1.transpose() * GRF;

    return tau;

}