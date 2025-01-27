#include "globals.h"

using namespace Eigen;
using namespace std;

double time_stop = 20;

// Global variables for all links
double joint_damping = 0.01;
Vector3d end_effector_trans;

// Base link
double base_mass;
Vector3d base_com_transformed;
Vector3d base_moments_of_inertia;
Vector3d base_products_of_inertia;
Matrix3d base_I_rotated_inertial;
Vector3d base_joint_axis;
Vector3d base_tran_pos_body;

// Shoulder link
double shoulder_mass;
Vector3d shoulder_com_transformed;
Vector3d shoulder_moments_of_inertia;
Vector3d shoulder_products_of_inertia;
Matrix3d shoulder_I_rotated_inertial;
Vector3d shoulder_joint_axis;
Vector3d shoulder_tran_pos_body;

// Upper arm link
double upper_arm_mass;
Vector3d upper_arm_com_transformed;
Vector3d upper_arm_moments_of_inertia;
Vector3d upper_arm_products_of_inertia;
Matrix3d upper_arm_I_rotated_inertial;
Vector3d upper_arm_axis;
Vector3d upper_arm_tran_pos_body;

// Upper forearm link
double upper_forearm_mass;
Vector3d upper_forearm_com_transformed;
Vector3d upper_forearm_moments_of_inertia;
Vector3d upper_forearm_products_of_inertia;
Matrix3d upper_forearm_I_rotated_inertial;
Vector3d upper_forearm_joint_axis;
Vector3d upper_forearm_tran_pos_body;

// Lower forearm link
double lower_forearm_mass;
Vector3d lower_forearm_com_transformed;
Vector3d lower_forearm_moments_of_inertia;
Vector3d lower_forearm_products_of_inertia;
Matrix3d lower_forearm_I_rotated_inertial;
Vector3d lower_forearm_joint_axis;
Vector3d lower_forearm_tran_pos_body;

// Wrist link
double wrist_mass;
Vector3d wrist_com_transformed;
Vector3d wrist_moments_of_inertia;
Vector3d wrist_products_of_inertia;
Matrix3d wrist_I_rotated_inertial;
Vector3d wrist_joint_axis;
Vector3d wrist_tran_pos_body;

// Gripper link
double gripper_link_mass;
Vector3d gripper_link_com_transformed;
Vector3d gripper_link_moments_of_inertia;
Vector3d gripper_link_products_of_inertia;
Matrix3d gripper_link_I_rotated_inertial;
Vector3d gripper_link_joint_axis;
Vector3d gripper_link_tran_pos_body;

// COM and MOI
Vector3d COM_b;
Matrix3d I_b;

// Joint Axis are in format (x, y, z) maybe
// Function to compute transformations for all links
void computeTransformations() {
    // End effector translation
    end_effector_trans = Eigen::Vector3d(0.02, 0, 0);
    
    // Base link
    base_mass = 0.538736;
    base_tran_pos_body = Eigen::Vector3d(0, 0, 0.058);  // Update global variable
    base_joint_axis = Eigen::Vector3d(0, 0, 0);         // Update global variable
    Eigen::Vector3d base_tran_pos_geom(0, 0, 0);
    Eigen::Vector4d base_tran_q_geom(1 / sqrt(2), 0, 0, 1 / sqrt(2));
    Eigen::Matrix3d base_MOI;
    base_MOI.setZero();
    base_MOI.diagonal() << 0.00252518, 0.00211519, 0.000690737;
    Eigen::Vector4d base_MOI_q(0.509292, 0.490887, -0.496359, 0.503269);
    Eigen::Vector3d base_com(-0.0380446, 0.000613892, 0.0193354);
    tie(base_com_transformed, base_moments_of_inertia, base_products_of_inertia, base_I_rotated_inertial) =
        transformInertialToGeom(base_com, base_MOI_q, base_MOI, base_tran_pos_geom, base_tran_q_geom);

    // Shoulder link
    shoulder_mass = 0.480879;
    shoulder_tran_pos_body = Eigen::Vector3d(0, 0, 0.072);  // Update global variable
    shoulder_joint_axis = Eigen::Vector3d(0, 0, 1);         // Update global variable
    Eigen::Vector3d shoulder_tran_pos_geom(0, 0, -0.003);
    Eigen::Vector4d shoulder_tran_q_geom(1 / sqrt(2), 0, 0, 1 / sqrt(2));
    Eigen::Matrix3d shoulder_MOI;
    shoulder_MOI.setZero();
    shoulder_MOI.diagonal() << 0.000588946, 0.000555655, 0.000378999;
    Eigen::Vector4d shoulder_MOI_q(0.0130352, 0.706387, 0.012996, 0.707586);
    Eigen::Vector3d shoulder_com(2.23482e-05, 4.14609e-05, 0.006628);
    tie(shoulder_com_transformed, shoulder_moments_of_inertia, shoulder_products_of_inertia, shoulder_I_rotated_inertial) =
        transformInertialToGeom(shoulder_com, shoulder_MOI_q, shoulder_MOI, shoulder_tran_pos_geom, shoulder_tran_q_geom);

    // Upper arm link
    upper_arm_mass = 0.430811;
    upper_arm_tran_pos_body = Eigen::Vector3d(0, 0, 0.03865);  // Update global variable
    upper_arm_axis = Eigen::Vector3d(0, 1, 0);                 // Update global variable
    Eigen::Vector3d upper_arm_tran_pos_geom(0, 0, 0);
    Eigen::Vector4d upper_arm_tran_q_geom(1 / sqrt(2), 0, 0, 1 / sqrt(2));
    Eigen::Matrix3d upper_arm_MOI;
    upper_arm_MOI.setZero();
    upper_arm_MOI.diagonal() << 0.00364425, 0.003463, 0.000399348;
    Eigen::Vector4d upper_arm_MOI_q(0.705539, 0.0470667, -0.0470667, 0.705539);
    Eigen::Vector3d upper_arm_com(0.0171605, 2.725e-07, 0.191323);
    tie(upper_arm_com_transformed, upper_arm_moments_of_inertia, upper_arm_products_of_inertia, upper_arm_I_rotated_inertial) =
        transformInertialToGeom(upper_arm_com, upper_arm_MOI_q, upper_arm_MOI, upper_arm_tran_pos_geom, upper_arm_tran_q_geom);

    // Upper forearm link
    upper_forearm_mass = 0.234589;
    upper_forearm_tran_pos_body = Eigen::Vector3d(0.04975, 0, 0.25);  // Update global variable
    upper_forearm_joint_axis = Eigen::Vector3d(0, 1, 0);              // Update global variable
    Eigen::Vector3d upper_forearm_tran_pos_geom(0, 0, 0);
    Eigen::Vector4d upper_forearm_tran_q_geom(0, 0, 0, 1);
    Eigen::Matrix3d upper_forearm_MOI;
    upper_forearm_MOI.setZero();
    upper_forearm_MOI.diagonal() << 0.000888, 0.000887807, 3.97035e-05;
    Eigen::Vector4d upper_forearm_MOI_q(0.000980829, 0.707106, -0.000980829, 0.707106);
    Eigen::Vector3d upper_forearm_com(0.107963, 0.000115876, 0);
    tie(upper_forearm_com_transformed, upper_forearm_moments_of_inertia, upper_forearm_products_of_inertia, upper_forearm_I_rotated_inertial) =
        transformInertialToGeom(upper_forearm_com, upper_forearm_MOI_q, upper_forearm_MOI, upper_forearm_tran_pos_geom, upper_forearm_tran_q_geom);

    // Lower forearm link
    lower_forearm_mass = 0.220991;
    lower_forearm_tran_pos_body = Eigen::Vector3d(0.175, 0, 0);  // Update global variable
    lower_forearm_joint_axis = Eigen::Vector3d(1, 0, 0);         // Update global variable
    Eigen::Vector3d lower_forearm_tran_pos_geom(0, 0, 0);
    Eigen::Vector4d lower_forearm_tran_q_geom(0, 1, 0, 0);
    Eigen::Matrix3d lower_forearm_MOI;
    lower_forearm_MOI.setZero();
    lower_forearm_MOI.diagonal() << 0.0001834, 0.000172527, 5.88633e-05;
    Eigen::Vector4d lower_forearm_MOI_q(-0.0732511, 0.703302, 0.0732511, 0.703302);
    Eigen::Vector3d lower_forearm_com(0.0374395, 0.00522252, 0);
    tie(lower_forearm_com_transformed, lower_forearm_moments_of_inertia, lower_forearm_products_of_inertia, lower_forearm_I_rotated_inertial) =
        transformInertialToGeom(lower_forearm_com, lower_forearm_MOI_q, lower_forearm_MOI, lower_forearm_tran_pos_geom, lower_forearm_tran_q_geom);
    
    // Wrist link
    wrist_mass = 0.084957;
    wrist_tran_pos_body = Eigen::Vector3d(0.075, 0, 0);  // Update global variable
    wrist_joint_axis = Eigen::Vector3d(0, 1, 0);         // Update global variable
    Eigen::Vector3d wrist_tran_pos_geom(0, 0, 0);
    Eigen::Vector4d wrist_tran_q_geom(1 / sqrt(2), 0, 0, 1 / sqrt(2));
    Eigen::Matrix3d wrist_MOI;
    wrist_MOI.setZero();
    wrist_MOI.diagonal() << 3.29057e-05, 3.082e-05, 2.68343e-05;
    Eigen::Vector4d wrist_MOI_q(0.608721, 0.363497, -0.359175, 0.606895);
    Eigen::Vector3d wrist_com(0.04236, -1.0663e-05, 0.010577);
    tie(wrist_com_transformed, wrist_moments_of_inertia, wrist_products_of_inertia, wrist_I_rotated_inertial) =
        transformInertialToGeom(wrist_com, wrist_MOI_q, wrist_MOI, wrist_tran_pos_geom, wrist_tran_q_geom);

    // Gripper link
    gripper_link_mass = 0.110084;
    gripper_link_tran_pos_body = Eigen::Vector3d(0.065, 0, 0);  // Update global variable
    gripper_link_joint_axis = Eigen::Vector3d(1, 0, 0);         // Update global variable
    Eigen::Vector3d gripper_link_tran_pos_geom(-0.02, 0, 0);
    Eigen::Vector4d gripper_link_tran_q_geom(1 / sqrt(2), 0, 0, 1 / sqrt(2));
    Eigen::Matrix3d gripper_link_MOI;
    gripper_link_MOI.setZero();
    gripper_link_MOI.diagonal() << 0.00307592, 0.00307326, 0.0030332;
    Eigen::Vector4d gripper_link_MOI_q(0.546081, 0.419626, 0.62801, 0.362371);
    Eigen::Vector3d gripper_link_com(0.0325296, 4.2061e-07, 0.0090959);
    tie(gripper_link_com_transformed, gripper_link_moments_of_inertia, gripper_link_products_of_inertia, gripper_link_I_rotated_inertial) =
        transformInertialToGeom(gripper_link_com, gripper_link_MOI_q, gripper_link_MOI, gripper_link_tran_pos_geom, gripper_link_tran_q_geom);

}

void computeInertialParams(const VectorXd qArm){
    std::pair<Vector3d, Matrix3d> result = inertial_params(qArm);
    // Extract the results as extern
    COM_b = result.first;
    I_b = result.second;

}