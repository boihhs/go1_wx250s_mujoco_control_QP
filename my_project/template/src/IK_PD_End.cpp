#include "IK_PD_End.h"
#include "Traj_Command_Goal.h"


using namespace Eigen;

VectorXd IK_PD_End(const VectorXd& xin, const double t, const VectorXd& qArm, const Vector3d& goal){
    
    VectorXd p_EF = xin.segment(13, 3);
    VectorXd V_EF = xin.segment(16, 3);
    Vector3d eul = xin.segment(0, 3); // roll, pitch, yaw
    Vector3d pos = xin.segment(3, 3);
    VectorXd q_IK = VectorXd::Zero(7);
    q_IK.segment(1, 6) = qArm; // Doing this so I don't have to change the long definations below
    //cout << q_IK.transpose();
    
    VectorXd xref = Traj_Command_01M(t, xin);
    //VectorXd xref = Traj_Command_Goal(t, xin, goal);

    VectorXd pe_des = xref.segment(13, 3);
    VectorXd ve_des = xref.segment(16, 3);

    Matrix3d R = eul2rotm(eul.reverse());
    //std::cout << "\n eul: \n" << eul << std::endl;

    Vector3d base = pos + R*(Vector3d(0, 0, .058) + Vector3d(0, 0, .072)); // Get pos of base of the arm
    
    Vector3d pEF_b_des = R.transpose()*(pe_des-base);
    //std::cout << "\n R: \n" << R << std::endl;
    //std::cout << "\n pe_des: \n" << pe_des << std::endl;
    //std::cout << "\n base: \n" << base << std::endl;
    
    Vector3d e_pb;
    e_pb << 1, 1, 1;

    
    Vector3d pEF_b;
    MatrixXd Jv(3, 6);
    MatrixXd Jw(3, 6);
    MatrixXd Jv_inv(Jv.cols(), Jv.rows());
    VectorXd dqArm(6);
    Matrix3d REF_b;
    Vector3d eulEF_b;
    double i = 0;
    while (e_pb.norm() > .002 and i < 100){
        i = i + 1;
        //std::cout << "\n i \n" << i << std::endl;
        pEF_b << cos(q_IK(1))*((199*cos(q_IK(2)))/4000 + sin(q_IK(2))/4) - (151*sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(4)) + cos(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))))/1000 + (151*cos(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))))/1000 + (cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)))/4 - (cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)))/4,
             sin(q_IK(1))*((199*cos(q_IK(2)))/4000 + sin(q_IK(2))/4) + (151*sin(q_IK(5))*(cos(q_IK(1))*sin(q_IK(4)) - cos(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))))/1000 - (151*cos(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))))/1000 - (sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)))/4 + (cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1)))/4,
             cos(q_IK(2)) / 4 - (199 * sin(q_IK(2))) / 4000 - (cos(q_IK(2)) * sin(q_IK(3))) / 4 - (cos(q_IK(3)) * sin(q_IK(2))) / 4 - (151 * cos(q_IK(5)) * (cos(q_IK(2)) * sin(q_IK(3)) + cos(q_IK(3)) * sin(q_IK(2)))) / 1000  - (151 * cos(q_IK(4)) * sin(q_IK(5)) * (cos(q_IK(2)) * cos(q_IK(3)) - sin(q_IK(2)) * sin(q_IK(3)))) / 1000 + 773 / 20000.0;        
        //std::cout << "\n pEF_b: \n" << std::endl;
        //std::cout << pEF_b << std::endl;
  
        REF_b = Rz(q_IK(1))*Ry(q_IK(2))*Ry(q_IK(3))*Rx(q_IK(4))*Ry(q_IK(5))*Rx(q_IK(6));
        //std::cout << "\n REF_b: \n" << REF_b << std::endl;
        eulEF_b = rotm2eul(REF_b).reverse(); // returns roll, pitch, yaw after reverse

        e_pb = pEF_b_des - pEF_b;
        //std::cout << "\n pEF_b_des: \n" << pEF_b_des << std::endl;
        //std::cout << "\n pEF_b: \n" << pEF_b << std::endl;
        
        Jv << (151*cos(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))))/1000 - (151*sin(q_IK(5))*(cos(q_IK(1))*sin(q_IK(4)) - cos(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))))/1000 - sin(q_IK(1))*((199*cos(q_IK(2)))/4000 + sin(q_IK(2))/4) + (sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)))/4 - (cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1)))/4, cos(q_IK(1))*(cos(q_IK(2))/4 - (199*sin(q_IK(2)))/4000) - (151*cos(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2))))/1000 - (151*cos(q_IK(4))*sin(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))))/1000 - (cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)))/4 - (cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))/4, - (151*cos(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2))))/1000 - (151*cos(q_IK(4))*sin(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))))/1000 - (cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)))/4 - (cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))/4, -(151*sin(q_IK(5))*(cos(q_IK(4))*sin(q_IK(1)) - sin(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))))/1000, - (151*cos(q_IK(5))*(sin(q_IK(1))*sin(q_IK(4)) + cos(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))))/1000 - (151*sin(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))))/1000, 0,
        cos(q_IK(1))*((199*cos(q_IK(2)))/4000 + sin(q_IK(2))/4) - (151*sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(4)) + cos(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))))/1000 + (151*cos(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))))/1000 + (cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)))/4 - (cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)))/4, sin(q_IK(1))*(cos(q_IK(2))/4 - (199*sin(q_IK(2)))/4000) - (151*cos(q_IK(5))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2))))/1000 + (151*cos(q_IK(4))*sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))))/1000 - (cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)))/4 - (cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))/4,   (151*cos(q_IK(4))*sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))))/1000 - (151*cos(q_IK(5))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2))))/1000 - (cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)))/4 - (cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))/4,  (151*sin(q_IK(5))*(cos(q_IK(1))*cos(q_IK(4)) + sin(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))))/1000,   (151*cos(q_IK(5))*(cos(q_IK(1))*sin(q_IK(4)) - cos(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))))/1000 + (151*sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))))/1000, 0,
                                                                                                                                                                                                                                                                                                                   0,                                                                  (sin(q_IK(2))*sin(q_IK(3)))/4 - sin(q_IK(2))/4 - (cos(q_IK(2))*cos(q_IK(3)))/4 - (199*cos(q_IK(2)))/4000 - (151*cos(q_IK(5))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3))))/1000 + (151*cos(q_IK(4))*sin(q_IK(5))*(cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(2))))/1000,                                                         (sin(q_IK(2))*sin(q_IK(3)))/4 - (cos(q_IK(2))*cos(q_IK(3)))/4 - (151*cos(q_IK(5))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3))))/1000 + (151*cos(q_IK(4))*sin(q_IK(5))*(cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(2))))/1000,                                          (151*sin(q_IK(4))*sin(q_IK(5))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3))))/1000,                                                             (151*sin(q_IK(5))*(cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(2))))/1000 - (151*cos(q_IK(4))*cos(q_IK(5))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3))))/1000, 0;
        //std::cout << "\n Jv: \n" << Jv << std::endl;
        Jw << 0, -sin(q_IK(1)), -sin(q_IK(1)), cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)), sin(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2))) - cos(q_IK(4))*sin(q_IK(1)), cos(q_IK(5))*(cos(q_IK(1))*cos(q_IK(2))*cos(q_IK(3)) - cos(q_IK(1))*sin(q_IK(2))*sin(q_IK(3))) - sin(q_IK(5))*(sin(q_IK(1))*sin(q_IK(4)) + cos(q_IK(4))*(cos(q_IK(1))*cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(1))*cos(q_IK(3))*sin(q_IK(2)))),
        0,  cos(q_IK(1)),  cos(q_IK(1)), cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1)) - sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)), cos(q_IK(1))*cos(q_IK(4)) + sin(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2))), sin(q_IK(5))*(cos(q_IK(1))*sin(q_IK(4)) - cos(q_IK(4))*(cos(q_IK(2))*sin(q_IK(1))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(1))*sin(q_IK(2)))) - cos(q_IK(5))*(sin(q_IK(1))*sin(q_IK(2))*sin(q_IK(3)) - cos(q_IK(2))*cos(q_IK(3))*sin(q_IK(1))),
        1,         0,         0,                 - cos(q_IK(2))*sin(q_IK(3)) - cos(q_IK(3))*sin(q_IK(2)),                                       sin(q_IK(4))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3))),                                                         - cos(q_IK(5))*(cos(q_IK(2))*sin(q_IK(3)) + cos(q_IK(3))*sin(q_IK(2))) - cos(q_IK(4))*sin(q_IK(5))*(cos(q_IK(2))*cos(q_IK(3)) - sin(q_IK(2))*sin(q_IK(3)));
        //std::cout << "\n Jw: \n" << Jw << std::endl;
        Jv_inv = Jv.transpose()*(Jv*Jv.transpose()).inverse();
        //std::cout << "\n Jv_inv: \n" << Jv_inv << std::endl;
        //std::cout << "\n e_pb: \n" << e_pb << std::endl;
        dqArm = Jv_inv*e_pb;
        //std::cout << "\n dqArm: \n" << dqArm << std::endl;
        q_IK.segment(1, 6) = q_IK.segment(1, 6) + dqArm;
        //std::cout << "\n q_IK.segment(1, 6): \n" << q_IK.segment(1, 6).transpose() << std::endl;
        
    }
    double factor = 1;
    MatrixXd kp = Eigen::MatrixXd::Zero(6, 6);
    kp.diagonal() << 70/factor, 80/factor, 70/factor, 15/factor, 10/factor, 0;
    //std::cout << "\n dqArm: \n" << dqArm << std::endl;
    //std::cout << "\n q_IK.segment(1, 6): \n" << q_IK.segment(1, 6) << std::endl;

    //std::cout << "\n qArm: \n" << qArm << std::endl;

    //std::cout << "\n kp: \n" << kp << std::endl;

    MatrixXd kd = Eigen::MatrixXd::Zero(6, 6);
    kd.diagonal() << 25/factor,   25/factor,  20/factor,  5/factor,   5/factor,   1/factor;
    //std::cout << "\n kd: \n" << kd << std::endl;

    //std::cout << "\n kp*(q_IK.segment(1, 6)-qArm): \n" << kp*(q_IK.segment(1, 6)-qArm) << std::endl;
    //std::cout << "\n kd*(-dqArm): \n" << kd*(-dqArm) << std::endl;

    //std::cout << "\n out: \n" << kp*(q_IK.segment(1, 6)-qArm)+kd*(-dqArm) << std::endl;
    return kp*(q_IK.segment(1, 6)-qArm)+kd*(-dqArm);



}