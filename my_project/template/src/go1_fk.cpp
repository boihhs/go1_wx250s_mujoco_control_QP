#include "go1_fk.h" // Include the header
#include <iostream>
#include <cmath>

using namespace Eigen;

Matrix4d transformationMatrices(const Matrix3d rotation, const double one, const double two, const double three){
  Matrix4d H_rot = Matrix4d::Identity();
  H_rot.block(0, 0, 3, 3) = rotation;

  Matrix4d H_trans = Matrix4d::Identity();
  H_trans(0,3) = one;
  H_trans(1,3) = two;
  H_trans(2,3) = three;
  return H_rot*H_trans;
  

}

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



  // Define vectors
  Eigen::Vector3d rhip(0.1881, 0.04675, 0.0);
  Eigen::Vector3d rthigh(0.0, 0.08, 0.0);
  Eigen::Vector3d rcalf(0.0, 0.0, -0.213);
  Eigen::Vector3d rfoot(0.0, 0.0, -0.213);

  Eigen::Vector3d r01 = Eigen::Vector3d(0, 0, 0.058) + Eigen::Vector3d(0, 0, 0.072); // [0, 0, 0.13]
  Eigen::Vector3d r12(0, 0, 0.03865);
  Eigen::Vector3d r23(0.04975, 0, 0.25);
  Eigen::Vector3d r34(0.175, 0, 0);
  Eigen::Vector3d r45(0.075, 0, 0);
  Eigen::Vector3d r56(0.065, 0, 0);
  Eigen::Vector3d r6o = Eigen::Vector3d(0.066, 0, 0) + Eigen::Vector3d(0.02, 0, 0); // [0.086, 0, 0]


  Eigen::Matrix4d H_hipFL = transformationMatrices(Rz(yaw) * Ry(pitch) * Rx(roll), rhip(0), rhip(1), -rhip(2));
  Eigen::Matrix4d H_hipFR = transformationMatrices(Rz(yaw) * Ry(pitch) * Rx(roll), rhip(0), -rhip(1), -rhip(2));
  Eigen::Matrix4d H_hipRL = transformationMatrices(Rz(yaw) * Ry(pitch) * Rx(roll), -rhip(0), rhip(1), -rhip(2));
  Eigen::Matrix4d H_hipRR = transformationMatrices(Rz(yaw) * Ry(pitch) * Rx(roll), -rhip(0), -rhip(1), -rhip(2));

  Eigen::Matrix4d H_thighFL = transformationMatrices(Rx(qFL0), rthigh(0), rthigh(1), rthigh(2));
  Eigen::Matrix4d H_thighFR = transformationMatrices(Rx(qFR0), rthigh(0), -rthigh(1), rthigh(2));
  Eigen::Matrix4d H_thighRL = transformationMatrices(Rx(qRL0), -rthigh(0), rthigh(1), rthigh(2));
  Eigen::Matrix4d H_thighRR = transformationMatrices(Rx(qRR0), -rthigh(0), -rthigh(1), rthigh(2));


  Eigen::Matrix4d H_calfFL = transformationMatrices(Ry(qFL1), rcalf(0), rcalf(1), rcalf(2));
  Eigen::Matrix4d H_calfFR = transformationMatrices(Ry(qFR1), rcalf(0), rcalf(1), rcalf(2));
  Eigen::Matrix4d H_calfRL = transformationMatrices(Ry(qRL1), rcalf(0), rcalf(1), rcalf(2));
  Eigen::Matrix4d H_calfRR = transformationMatrices(Ry(qRR1), rcalf(0), rcalf(1), rcalf(2));



  Eigen::Matrix4d H_footFL = transformationMatrices(Ry(qFL2), rfoot(0), rfoot(1), rfoot(2));
  Eigen::Matrix4d H_footFR = transformationMatrices(Ry(qFR2), rfoot(0), rfoot(1), rfoot(2));
  Eigen::Matrix4d H_footRL = transformationMatrices(Ry(qRL2), rfoot(0), rfoot(1), rfoot(2));
  Eigen::Matrix4d H_footRR = transformationMatrices(Ry(qRR2), rfoot(0), rfoot(1), rfoot(2));


  // Construct H01
  Eigen::Matrix4d H01 = transformationMatrices(Rz(yaw) * Ry(pitch) * Rx(roll), r01(0), r01(1), r01(2));
  Eigen::Matrix4d H12 = transformationMatrices(Rz(qa1), r12(0), r12(1), r12(2));
  Eigen::Matrix4d H23 = transformationMatrices(Ry(qa2), r23(0), r23(1), r23(2));
  Eigen::Matrix4d H34 = transformationMatrices(Ry(qa3), r34(0), r34(1), r34(2));
  Eigen::Matrix4d H45 = transformationMatrices(Rx(qa4), r45(0), r45(1), r45(2));
  Eigen::Matrix4d H56 = transformationMatrices(Ry(qa5), r56(0), r56(1), r56(2));
  Eigen::Matrix4d H6o = transformationMatrices(Rx(qa6), r6o(0), r6o(1), r6o(2));

  // Compute foot positions
  Eigen::Vector4d noHomo(0, 0, 0, 1);

  Eigen::Vector4d footFL_homog = H_hipFL * H_thighFL * H_calfFL * H_footFL * noHomo;
  Eigen::Vector3d footFL = footFL_homog.head<3>() + xin.segment(3, 3);

  Eigen::Vector4d footFR_homog = H_hipFR * H_thighFR * H_calfFR * H_footFR * noHomo;
  Eigen::Vector3d footFR = footFR_homog.head<3>() + xin.segment(3, 3);

  Eigen::Vector4d footRL_homog = H_hipRL * H_thighRL * H_calfRL * H_footRL * noHomo;
  Eigen::Vector3d footRL = footRL_homog.head<3>() + xin.segment(3, 3);

  Eigen::Vector4d footRR_homog = H_hipRR * H_thighRR * H_calfRR * H_footRR * noHomo;
  Eigen::Vector3d footRR = footRR_homog.head<3>() + xin.segment(3, 3);

  // Compute end effector position
  Eigen::Matrix4d H_EF = H01 * H12 * H23 * H34 * H45 * H56 * H6o;
  Eigen::Vector4d EF_homog = H_EF * noHomo;
  Eigen::Vector3d EF = xin.segment(3, 3) + EF_homog.head<3>();

  // Assign positions
    VectorXd posVel(30);

    posVel.segment(0, 3) = footFL;
    posVel.segment(6, 3) = footFR;
    posVel.segment(12, 3) = footRL;
    posVel.segment(18, 3) = footRR;
    posVel.segment(24, 3) = EF;
    return posVel;
}

