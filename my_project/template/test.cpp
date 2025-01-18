#include "MPC_QP.h"
#include <iostream>
#include <Eigen/Dense>
int main() {
    // Example Usage
    Eigen::VectorXd xin(19);
    xin.setZero();
    xin(5) = .3;
    xin(12) = 1;


    Eigen::VectorXd PosVel(30);
    PosVel.setZero();
    PosVel(0) = 0.1;
    PosVel(1) = 0.2;
    PosVel(2) = 0.3;
    PosVel(6) = 1;
    PosVel(7) = 2;
    PosVel(8) = 3;
    PosVel(12) = 0.1;
    PosVel(13) = 0.2;
    PosVel(14) = 0.3;
    PosVel(18) = 1;
    PosVel(19) = 2;
    PosVel(20) = 3;
    PosVel(24) = 0.1;
    PosVel(25) = 0.2;
    PosVel(26) = 0.3;

    Eigen::VectorXd qArm(6);
    qArm.setZero();
    Eigen::Vector3d F_Tatile(0, 0, 0);
    double t = 1.0;

    Eigen::VectorXd F_c = MPC_QP(xin, PosVel, qArm, F_Tatile, t);

    std::cout << "F_c: " << F_c << std::endl;

    return 0;
}