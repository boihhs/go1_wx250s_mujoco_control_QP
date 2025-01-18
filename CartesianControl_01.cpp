#include "CartesianControl_01.h"

//computeLegJacobianAndPosition implementation:
std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeLegJacobianAndPosition(const Eigen::Vector3d& q, int leg) {
    double l1 = 0.08; // hip length
    double l2 = 0.213; // thigh length
    double l3 = 0.213; // calf length

    double sideSign = (leg == 1 || leg == 3) ? -1.0 : 1.0;

    double s1 = std::sin(q(0)); // for hip joint
    double s2 = std::sin(q(1)); // for thigh joint
    double s3 = std::sin(q(2)); // for calf joint

    double c1 = std::cos(q(0)); // for hip joint
    double c2 = std::cos(q(1)); // for thigh joint
    double c3 = std::cos(q(2)); // for calf joint

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    J(0, 0) = 0.0;
    J(1, 0) = -sideSign * l1 * s1 + l2 * c2 * c1 + l3 * c23 * c1;
    J(2, 0) = sideSign * l1 * c1 + l2 * c2 * s1 + l3 * c23 * s1;

    J(0, 1) = -l3 * c23 - l2 * c2;
    J(1, 1) = -l2 * s2 * s1 - l3 * s23 * s1;
    J(2, 1) = l2 * s2 * c1 + l3 * s23 * c1;

    J(0, 2) = -l3 * c23;
    J(1, 2) = -l3 * s23 * s1;
    J(2, 2) = l3 * s23 * c1;

    Eigen::Vector3d p;
    p(0) = -l3*s23 - l2*s2; // in x direction
    p(1) = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1; // in y direction
    p(2) = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2; // in z direction
    
    return std::make_pair(J,p);
}

//LegTorque_quadruped implementation:
Eigen::VectorXd LegTorque_quadruped(const std::vector<Eigen::Vector3d>& F, const Eigen::VectorXd& q, double t) {
        Eigen::Matrix3d J1, J2, J3, J4;
        Eigen::Vector3d p1, p2, p3, p4;
    std::tie(J1,p1) = computeLegJacobianAndPosition(q.segment<3>(0), 1);
    Eigen::Vector3d tau1 = J1.transpose() * F[0];

    std::tie(J2,p2) = computeLegJacobianAndPosition(q.segment<3>(3), 2);
    Eigen::Vector3d tau2 = J2.transpose() * F[1];

    std::tie(J3,p3) = computeLegJacobianAndPosition(q.segment<3>(6), 3);
    Eigen::Vector3d tau3 = J3.transpose() * F[2];

    std::tie(J4,p4) = computeLegJacobianAndPosition(q.segment<3>(9), 4);
    Eigen::Vector3d tau4 = J4.transpose() * F[3];

    Eigen::VectorXd tau(12);
    tau << tau1, tau2, tau3, tau4;
    return tau;
}


Eigen::VectorXd CartesianControl_01(double t, const Eigen::VectorXd& xin, const Eigen::VectorXd& q, const Eigen::VectorXd& xdes, const Eigen::VectorXd& fpfv, const Eigen::VectorXd& qArm) {
    double gaitcycle = 0.4;
    double t_start = 0;
    double dt = .04;
    Eigen::Matrix3d R = quat2rotm(xin.segment<4>(0)/xin.segment<4>(0).norm());

    Eigen::Vector3d rpy_act = rotm2eul(R); // converts to yaw pitch roll
    double roll_act = rpy_act(2);
    double pitch_act = rpy_act(1);
    double yaw_act = rpy_act(0);
    Eigen::Matrix3d R_hip = eul2rotm(Eigen::Vector3d(yaw_act,pitch_act,0));
    double x_act = xin(4);
    double y_act = xin(5);
    double z_act = xin(6);
    double vx_des = xdes(10);
    double vy_des = xdes(11);
    double vx_act = xin(10);
    double vy_act = xin(11);

    double lift_height = 0.1;

    Eigen::Vector3d fpFL = fpfv.segment<3>(0);
    Eigen::Vector3d fpFR = fpfv.segment<3>(6);
    Eigen::Vector3d fpRL = fpfv.segment<3>(12);
    Eigen::Vector3d fpRR = fpfv.segment<3>(18);

    Eigen::Vector3d fvFL = fpfv.segment<3>(3);
    Eigen::Vector3d fvFR = fpfv.segment<3>(9);
    Eigen::Vector3d fvRL = fpfv.segment<3>(15);
    Eigen::Vector3d fvRR = fpfv.segment<3>(21);

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> result = inertial_params(qArm);
    // Extract the results
    Eigen::Vector3d COM_b = result.first;
    Eigen::Matrix3d I_b = result.second;

    

    Eigen::Vector3d p_COM = xin.segment<3>(4) + R * COM_b;

    Eigen::VectorXd tau;


    if (t > t_start) {
        double t_cycle = std::floor(std::fmod(t - t_start, gaitcycle / 2.0));
        int i_cycle = std::floor((t - t_start) / (gaitcycle / 2.0)) + 1;
        int i_gait = i_cycle % 2;
        Eigen::Vector4d swing_schedule = Eigen::Vector4d::Zero();
        
        if (i_gait == 0) { //trotting gait
            swing_schedule(0) = 1;
            swing_schedule(1) = 0;
            swing_schedule(2) = 0;
            swing_schedule(3) = 1;
        } else {
            swing_schedule(0) = 0;
            swing_schedule(1) = 1;
            swing_schedule(2) = 1;
            swing_schedule(3) = 0;
        }
        
        //bounding gait
        // if (i_gait == 0) {
        //     swing_schedule(0) = 0;
        //     swing_schedule(1) = 0;
        //     swing_schedule(2) = 1;
        //     swing_schedule(3) = 1;
        // } else {
        //     swing_schedule(0) = 1;
        //     swing_schedule(1) = 1;
        //     swing_schedule(2) = 0;
        //     swing_schedule(3) = 0;
        // }

        double fzdes = -(lift_height / std::pow(gaitcycle / 4.0, 2.0)) * t_cycle * (t_cycle - gaitcycle / 2.0) - 0.0;
        
        double dfzdes = -2 * (lift_height / std::pow(gaitcycle / 4.0, 2.0)) * t_cycle + (lift_height / std::pow(gaitcycle / 4.0, 2.0)) * gaitcycle / 2.0;
        

        double delta_t = gaitcycle / 2.0;

        Eigen::Vector3d p_hip_FL_w = p_COM + R * Eigen::Vector3d(0.1881, 0.04675, 0) + R_hip * Eigen::Vector3d(0, 0.08, 0);
        Eigen::Vector3d p_hip_FR_w = p_COM + R * Eigen::Vector3d(0.1881, -0.04675, 0) + R_hip * Eigen::Vector3d(0, -0.08, 0);
        Eigen::Vector3d p_hip_RL_w = p_COM + R * Eigen::Vector3d(-0.1881, 0.04675, 0) + R_hip * Eigen::Vector3d(0, 0.08, 0);
        Eigen::Vector3d p_hip_RR_w = p_COM + R * Eigen::Vector3d(-0.1881, -0.04675, 0) + R_hip * Eigen::Vector3d(0, -0.08, 0);

        double k = 0.03;
        double fx_des_FL = p_hip_FL_w(0) + delta_t * vx_act / 2.0 + k * (vx_act - vx_des);
        double fy_des_FL = p_hip_FL_w(1) + delta_t * vy_act / 2.0 + k * (vy_act - vy_des);
        double fx_des_FR = p_hip_FR_w(0) + delta_t * vx_act / 2.0 + k * (vx_act - vx_des);
        double fy_des_FR = p_hip_FR_w(1) + delta_t * vy_act / 2.0 + k * (vy_act - vy_des);
        double fx_des_RL = p_hip_RL_w(0) + delta_t * vx_act / 2.0 + k * (vx_act - vx_des);
        double fy_des_RL = p_hip_RL_w(1) + delta_t * vy_act / 2.0 + k * (vy_act - vy_des);
        double fx_des_RR = p_hip_RR_w(0) + delta_t * vx_act / 2.0 + k * (vx_act - vx_des);
        double fy_des_RR = p_hip_RR_w(1) + delta_t * vy_act / 2.0 + k * (vy_act - vy_des);
        
        Eigen::Matrix3d Kp = (Eigen::Matrix3d() << 250, 0, 0, 0, 250, 0, 0, 0, 100).finished();
        Eigen::Matrix3d Kd = (Eigen::Matrix3d() << 25, 0, 0, 0, 25, 0, 0, 0, 10).finished();
        
        Eigen::Vector3d F_FL = Kp * (Eigen::Vector3d(fx_des_FL, fy_des_FL, fzdes) - fpFL) + Kd * (Eigen::Vector3d(0, 0, dfzdes) - fvFL);
        F_FL = F_FL * swing_schedule(0);
        Eigen::Vector3d F_FR = Kp * (Eigen::Vector3d(fx_des_FR, fy_des_FR, fzdes) - fpFR) + Kd * (Eigen::Vector3d(0, 0, dfzdes) - fvFR);
        F_FR = F_FR * swing_schedule(1);
        Eigen::Vector3d F_RL = Kp * (Eigen::Vector3d(fx_des_RL, fy_des_RL, fzdes) - fpRL) + Kd * (Eigen::Vector3d(0, 0, dfzdes) - fvRL);
        F_RL = F_RL * swing_schedule(2);
        Eigen::Vector3d F_RR = Kp * (Eigen::Vector3d(fx_des_RR, fy_des_RR, fzdes) - fpRR) + Kd * (Eigen::Vector3d(0, 0, dfzdes) - fvRR);
        F_RR = F_RR * swing_schedule(3);
      
        std::vector<Eigen::Vector3d> forces_body = {R.transpose()*F_FL,R.transpose()*F_FR,R.transpose()*F_RL,R.transpose()*F_RR};
        //std::cout << forces_body << std::endl;
        tau = LegTorque_quadruped(forces_body, q, t);
        
        // std::vector<Eigen::Vector3d> forces_world = {F_FL,F_FR,F_RL,F_RR};
        // tau = LegTorque_quadruped(forces_world,q,t);

    } else {
        tau = Eigen::VectorXd::Zero(12);
    }

    return tau;
}