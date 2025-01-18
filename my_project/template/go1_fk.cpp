#include "go1_fk.h" // Include the header
#include <iostream>
#include <cmath>

using namespace Eigen;

// VectorXd FK(const VectorXd& xin, const VectorXd& q, const VectorXd& qArm, const Eigen::VectorXd& dq, const Eigen::VectorXd& dq_arm) {
VectorXd FK(const VectorXd& xin, const VectorXd& q, const VectorXd& qArm) {
    double roll = xin(0);
    double pitch = xin(1);
    double yaw = xin(2);
    // Extract joint angles
    double qFL0 = q(0);  double qFL1 = q(1);  double qFL2 = q(2);
    double qFR0 = q(3);  double qFR1 = q(4);  double qFR2 = q(5);
    double qRL0 = q(6);  double qRL1 = q(7);  double qRL2 = q(8);
    double qRR0 = q(9);  double qRR1 = q(10); double qRR2 = q(11);

    // Extract arm joint angles
    double qa1 = qArm(0); double qa2 = qArm(1); double qa3 = qArm(2);
    double qa4 = qArm(3); double qa5 = qArm(4); double qa6 = qArm(5);



    // Link lengths
    Vector3d rhip(0.1881, 0.04675, 0);
    Vector3d rthigh(0, 0.08, 0);
    Vector3d rcalf(0, 0, -0.213);
    Vector3d rfoot(0, 0, -0.213);
    
    Vector3d r01(0, 0, 0.058);
    Vector3d r12(0, 0, 0.072);
    Vector3d r23(0, 0, 0.03865);
    Vector3d r34(0.04975, 0, 0.25);
    Vector3d r45(0.175, 0, 0);
    Vector3d r56(0.075, 0, 0);
    Vector3d r6o(0.065, 0, 0);
    r6o = r6o + Vector3d(0.02, 0, 0);

    Matrix4d H_hipFL = Matrix4d::Identity();
    Matrix4d H_hipFR = Matrix4d::Identity();
    Matrix4d H_hipRL = Matrix4d::Identity();
    Matrix4d H_hipRR = Matrix4d::Identity();

    Matrix4d H_thighFL = Matrix4d::Identity();
    Matrix4d H_thighFR = Matrix4d::Identity();
    Matrix4d H_thighRL = Matrix4d::Identity();
    Matrix4d H_thighRR = Matrix4d::Identity();

    Matrix4d H_calfFL = Matrix4d::Identity();
    Matrix4d H_calfFR = Matrix4d::Identity();
    Matrix4d H_calfRL = Matrix4d::Identity();
    Matrix4d H_calfRR = Matrix4d::Identity();

    Matrix4d H_footFL = Matrix4d::Identity();
    Matrix4d H_footFR = Matrix4d::Identity();
    Matrix4d H_footRL = Matrix4d::Identity();
    Matrix4d H_footRR = Matrix4d::Identity();

    Matrix4d H01 = Matrix4d::Identity();
    Matrix4d H12 = Matrix4d::Identity();
    Matrix4d H23 = Matrix4d::Identity();
    Matrix4d H34 = Matrix4d::Identity();
    Matrix4d H45 = Matrix4d::Identity();
    Matrix4d H56 = Matrix4d::Identity();
    Matrix4d H6o = Matrix4d::Identity();


   // Construct rotation matrices from yaw, pitch, roll
    Matrix3d R_body = Rz(yaw) * Ry(pitch) * Rx(roll);

    // Define a 4x4 identity matrix
    Matrix4d I = Matrix4d::Identity();
    
    // Homogeneous Transforms - **Key change here: Rotation THEN Translation**
    H_hipFL.block<3, 3>(0, 0) = R_body;
    H_hipFL.block<3, 1>(0, 3) = R_body * rhip;
    H_hipFL(3,3) = 1;

     H_hipFR.block<3, 3>(0, 0) = R_body;
    H_hipFR.block<3, 1>(0, 3) = R_body * Vector3d(rhip(0), -rhip(1), -rhip(2));
    H_hipFR(3,3) = 1;

    H_hipRL.block<3, 3>(0, 0) = R_body;
    H_hipRL.block<3, 1>(0, 3) = R_body * Vector3d(-rhip(0), rhip(1), -rhip(2));
    H_hipRL(3,3) = 1;
   
    H_hipRR.block<3, 3>(0, 0) = R_body;
    H_hipRR.block<3, 1>(0, 3) = R_body * Vector3d(-rhip(0), -rhip(1), -rhip(2));
    H_hipRR(3,3) = 1;
    
   
   
    H_thighFL.block<3, 3>(0, 0) = Rx(qFL0);
    H_thighFL.block<3, 1>(0, 3) = Rx(qFL0) * rthigh;
    H_thighFL(3,3) = 1;


    H_thighFR.block<3, 3>(0, 0) = Rx(qFR0);
    H_thighFR.block<3, 1>(0, 3) = Rx(qFR0) * Vector3d(rthigh(0), -rthigh(1), rthigh(2));
   H_thighFR(3,3) = 1;


    H_thighRL.block<3, 3>(0, 0) = Rx(qRL0);
    H_thighRL.block<3, 1>(0, 3) = Rx(qRL0) * Vector3d(-rthigh(0), rthigh(1), rthigh(2));
   H_thighRL(3,3) = 1;


    H_thighRR.block<3, 3>(0, 0) = Rx(qRR0);
    H_thighRR.block<3, 1>(0, 3) = Rx(qRR0) * Vector3d(-rthigh(0), -rthigh(1), rthigh(2));
   H_thighRR(3,3) = 1;
    
    H_calfFL.block<3, 3>(0, 0) = Ry(qFL1);
     H_calfFL.block<3, 1>(0, 3) = Ry(qFL1) * rcalf;
   H_calfFL(3,3) = 1;


    H_calfFR.block<3, 3>(0, 0) = Ry(qFR1);
    H_calfFR.block<3, 1>(0, 3) = Ry(qFR1) * rcalf;
   H_calfFR(3,3) = 1;

    H_calfRL.block<3, 3>(0, 0) = Ry(qRL1);
    H_calfRL.block<3, 1>(0, 3) = Ry(qRL1) * rcalf;
   H_calfRL(3,3) = 1;
    

    H_calfRR.block<3, 3>(0, 0) = Ry(qRR1);
    H_calfRR.block<3, 1>(0, 3) = Ry(qRR1) * rcalf;
   H_calfRR(3,3) = 1;

    H_footFL.block<3, 3>(0, 0) = Ry(qFL2);
     H_footFL.block<3, 1>(0, 3) = Ry(qFL2) * rfoot;
    H_footFL(3,3) = 1;
    
    H_footFR.block<3, 3>(0, 0) = Ry(qFR2);
   H_footFR.block<3, 1>(0, 3) = Ry(qFR2) * rfoot;
     H_footFR(3,3) = 1;

    H_footRL.block<3, 3>(0, 0) = Ry(qRL2);
   H_footRL.block<3, 1>(0, 3) = Ry(qRL2) * rfoot;
     H_footRL(3,3) = 1;

    H_footRR.block<3, 3>(0, 0) = Ry(qRR2);
   H_footRR.block<3, 1>(0, 3) = Ry(qRR2) * rfoot;
   H_footRR(3,3) = 1;


   H01.block<3,3>(0,0) = R_body;
    H01.block<3,1>(0,3) = R_body * r01;
    H01(3,3) = 1;


    H12.block<3,3>(0,0) = Rz(qa1);
    H12.block<3,1>(0,3) = Rz(qa1) * r12;
    H12(3,3) = 1;


    H23.block<3,3>(0,0) = Ry(qa2);
    H23.block<3,1>(0,3) = Ry(qa2) * r23;
    H23(3,3) = 1;


    H34.block<3,3>(0,0) = Ry(qa3);
    H34.block<3,1>(0,3) = Ry(qa3) * r34;
    H34(3,3) = 1;


    H45.block<3,3>(0,0) = Rx(qa4);
    H45.block<3,1>(0,3) = Rx(qa4) * r45;
    H45(3,3) = 1;


    H56.block<3,3>(0,0) = Ry(qa5);
    H56.block<3,1>(0,3) = Ry(qa5) * r56;
    H56(3,3) = 1;


    H6o.block<3,3>(0,0) = Rx(qa6);
    H6o.block<3,1>(0,3) = Rx(qa6) * r6o;
    H6o(3,3) = 1;
    
    // Positions (using homogeneous coordinates)
    Vector4d footFL_h = H_hipFL * H_thighFL * H_calfFL * H_footFL * Vector4d(0, 0, 0, 1);
    std::cout << H_hipFL;
    Vector3d FL = footFL_h.head(3) + xin.segment(3, 3);

    Vector4d footFR_h = H_hipFR * H_thighFR * H_calfFR * H_footFR * Vector4d(0, 0, 0, 1);
    Vector3d FR = footFR_h.head(3) + xin.segment(3, 3);

    Vector4d footRL_h = H_hipRL * H_thighRL * H_calfRL * H_footRL * Vector4d(0, 0, 0, 1);
    Vector3d RL = footRL_h.head(3) + xin.segment(3, 3);

    Vector4d footRR_h = H_hipRR * H_thighRR * H_calfRR * H_footRR * Vector4d(0, 0, 0, 1);
    Vector3d RR = footRR_h.head(3) + xin.segment(3, 3);


    Vector4d H_EF_h = H01*H12*H23*H34*H45*H56*H6o * Vector4d(0,0,0,1);
    Vector3d EF = H_EF_h.head(3) + xin.segment(3, 3);

    // Assign positions
    VectorXd posVel(30);

    posVel.segment(0, 3) = FL;
    posVel.segment(6, 3) = FR;
    posVel.segment(12, 3) = RL;
    posVel.segment(18, 3) = RR;
    posVel.segment(24, 3) = EF;
    
    return posVel;
}