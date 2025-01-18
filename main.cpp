#include "MPC_QP.h"
#include <iostream>
#include "go1_fk.h"
#include "Cartesian_PD_End.h"
#include "ContactJacobian_Arm.h"
#include "contact_jacobian.h"
#include "utils.h"

#include <Eigen/Dense>

int main() {
    // Example Usage
    Eigen::VectorXd xin(19);
    xin.setZero();

    Eigen::VectorXd x(19);
    //std::cout << "\n\n Eular: " << eular.transpose();

    Eigen::VectorXd qArm(6);

    Eigen::VectorXd q(12);
    Eigen::VectorXd F(15);

    //Eigen::VectorXd dqArm(6);

    /*
    x << 0.999988, 7.94711e-07, 0.00491891, -4.24742e-06, -0.000744704, 4.12325e-07, 0.443993, 7.83053e-05, 1.05274, -0.0012436, -0.0824663, 2.80328e-05, -0.134444, 0, 0, 0, 0, 0, 0;

    q << -0.00318461, 0.0428765, -0.20866, 0.00318008, 0.0428777, -0.20866, 0.00125415, 0.0381618, -0.209407, -0.00129734, 0.0381663, -0.209407;

    dq << 2.46776, -15.7017, -0.152412, 2.46802, -15.7017, -0.269165, 1.82968, -15.7594, -5.05475e-05, 1.83034, -15.7594, 0.148147;

    qArm << -6.75991e-07, -0.00606093, -0.00127515, 5.32795e-07, -0.000237755, 0;

    */

    x << 0.999937, 0.000316117, 0.0112371, 0.000214365, -0.00534712, -0.000253453,
     0.346666, 0.023176, 0.185211, 0.00698819, -0.085983, -0.00265304,
     -1.37847, 0, 0, 0, 0, 0, 0;


    //F << 23,    1.0995,   39.7640,    1.9578,    1.0968,   40.3946,    0.9465,   -0.6450,   38.9634,    1.9628,   -0.6453,   39.6007,    0.0023,   -0.0001,   -0.3517;
    qArm.setZero();
    qArm << 1.11555e-05, 0.0041293, 0.0018829, -4.7202e-06, 0.000591314, -6.76673e-07;
    q.setZero();
    q << 0.210702, 1.35548, -2.61986, -0.209174, 1.39137, -2.62472,
     0.193458, 1.36204, -2.6253, -0.191431, 1.39046, -2.62969;
;
    
    Vector4d quat = x.segment(0, 4);
    Vector3d eular = rotm2eul(quat2rotm(quat/quat.norm()));
    xin.segment(0, 3) = eular.reverse();
    xin.segment(3, 9) = x.segment(4, 9);
    xin(12) = 1;

    Eigen::Vector3d F_Tatile(0, 0, 0);

    double t = 11.6970;

    Eigen::VectorXd PosVel = FK(xin, q, qArm);

    xin.segment(13, 6) = PosVel.segment(24, 6);
    x.segment(13, 6) = PosVel.segment(24, 6);

    std::cout << "\n xin: " << xin << std::endl;    
    F = MPC_QP(xin, PosVel, qArm, F_Tatile, t);
    Eigen::VectorXd FArm1 = Cartesian_PD_End(x, t);
    
    Eigen::VectorXd FArm = FArm1 + F.segment(12,3);

    

    Eigen::VectorXd F_c = F.segment(0,12);
    Eigen::VectorXd tauMain = ContactJacobian_01(x, q, F_c);
    Eigen::VectorXd tau(18);
    tau.setZero();
    tau.segment(0,12) = tauMain.segment(6,12);
    //Eigen::VectorXd tauArm = ContactJacobian_Arm(x,qArm,FArm);
    std::cout << "\n F: " << F.transpose();
    //tau.segment(12, 6) = tauArm;
    
    std::cout << "tau: " << tau.transpose() << std::endl;    
    return 0;
}