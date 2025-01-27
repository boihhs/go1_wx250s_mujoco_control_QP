#include "ContactJacobian_Arm.h"

Eigen::VectorXd ContactJacobian_Arm(const Eigen::VectorXd& xin, const Eigen::VectorXd& q_arm, const Eigen::VectorXd& dq_arm, const Eigen::VectorXd& F_arm) {
    
    Eigen::MatrixXd JvEF = ContactJacobianArm(xin, q_arm);
    Eigen::VectorXd tau = JvEF.transpose() * -F_arm;

    Eigen::VectorXd tau_damp = 5*-dq_arm;

    Eigen::VectorXd tau_arm = tau.segment(6,6)+tau_damp;

    return tau_arm;

}