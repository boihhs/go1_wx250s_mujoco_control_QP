#include "Cartesian_PD_End.h"
using namespace Eigen;

VectorXd Cartesian_PD_End(const VectorXd& x, double t) {
    Vector3d p_EF = x.segment<3>(13);
    Vector3d v_EF = x.segment<3>(16);

    //Eigen::VectorXd xref = Traj_Command_01M(t, x);
    Eigen::VectorXd xref = Traj_Command_01M(t);

    Vector3d p_des = xref.segment<3>(13);
    Vector3d v_des = xref.segment<3>(16);

    Matrix3d Kp = Matrix3d::Zero();
    Kp.diagonal() << 50, 50, 50;

    Matrix3d Kd = Matrix3d::Zero();
    Kd.diagonal() << 15, 15, 15;

    Vector3d F_PD = Kp * (p_des - p_EF) + Kd * (v_des - v_EF);
    return -F_PD;
}
