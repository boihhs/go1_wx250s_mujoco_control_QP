#include "getVeloctyArm_feet.h"



Eigen::VectorXd legVelocity(const Eigen::VectorXd& xin, const Eigen::VectorXd& q, const Eigen::VectorXd& dq){
    Eigen::VectorXd dx(18);
    dx.setZero();
    Eigen::MatrixXd Jc1 = ContactJacobian(xin, q);
    Eigen::Matrix3d R_r = eul2rotm(xin.segment(0,3));
    

    Eigen::VectorXd deular = R_r.transpose()*xin.segment(6, 3);

    dx.segment(0, 3) = xin.segment(9, 3);
    dx.segment(3, 3) = deular;
    dx.segment(6, 12) = dq;

    return Jc1*dx;


}

Eigen::Vector3d armVelocity(const Eigen::VectorXd& xin, const Eigen::VectorXd& q_arm, const Eigen::VectorXd& dq_arm){
    Eigen::MatrixXd JvEF = ContactJacobianArm(xin, q_arm);
    Eigen::VectorXd dx(12);
    dx.setZero();
    Eigen::Matrix3d R_r = eul2rotm(xin.segment(0,3));
    

    Eigen::VectorXd deular = R_r.transpose()*xin.segment(6, 3);

    dx.segment(0, 3) = xin.segment(9, 3);
    dx.segment(3, 3) = deular;
    dx.segment(6, 6) = dq_arm;
    
    return JvEF*dx;

}