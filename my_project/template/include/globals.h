#ifndef GLOBALS_H
#define GLOBALS_H

#include <Eigen/Dense>
#include <tuple>
#include "transform_inertial_to_geom.h"
#include "inertial_params.h"


extern Eigen::Vector3d COM_b;
extern Eigen::Matrix3d I_b;

extern double time_stop;
// Declare global variables for all links
extern double joint_damping;

// Base link
extern double base_mass;
extern Eigen::Vector3d base_com_transformed;
extern Eigen::Vector3d base_moments_of_inertia;
extern Eigen::Vector3d base_products_of_inertia;
extern Eigen::Matrix3d base_I_rotated_inertial;
extern Eigen::Vector3d base_joint_axis;
extern Eigen::Vector3d base_tran_pos_body;

// Shoulder link
extern double shoulder_mass;
extern Eigen::Vector3d shoulder_com_transformed;
extern Eigen::Vector3d shoulder_moments_of_inertia;
extern Eigen::Vector3d shoulder_products_of_inertia;
extern Eigen::Matrix3d shoulder_I_rotated_inertial;
extern Eigen::Vector3d shoulder_joint_axis;
extern Eigen::Vector3d shoulder_tran_pos_body;

// Upper arm link
extern double upper_arm_mass;
extern Eigen::Vector3d upper_arm_com_transformed;
extern Eigen::Vector3d upper_arm_moments_of_inertia;
extern Eigen::Vector3d upper_arm_products_of_inertia;
extern Eigen::Matrix3d upper_arm_I_rotated_inertial;
extern Eigen::Vector3d upper_arm_axis;
extern Eigen::Vector3d upper_arm_tran_pos_body;

// Upper forearm link
extern double upper_forearm_mass;
extern Eigen::Vector3d upper_forearm_com_transformed;
extern Eigen::Vector3d upper_forearm_moments_of_inertia;
extern Eigen::Vector3d upper_forearm_products_of_inertia;
extern Eigen::Matrix3d upper_forearm_I_rotated_inertial;
extern Eigen::Vector3d upper_forearm_joint_axis;
extern Eigen::Vector3d upper_forearm_tran_pos_body;

// Lower forearm link
extern double lower_forearm_mass;
extern Eigen::Vector3d lower_forearm_com_transformed;
extern Eigen::Vector3d lower_forearm_moments_of_inertia;
extern Eigen::Vector3d lower_forearm_products_of_inertia;
extern Eigen::Matrix3d lower_forearm_I_rotated_inertial;
extern Eigen::Vector3d lower_forearm_joint_axis;
extern Eigen::Vector3d lower_forearm_tran_pos_body;

// Wrist link
extern double wrist_mass;
extern Eigen::Vector3d wrist_com_transformed;
extern Eigen::Vector3d wrist_moments_of_inertia;
extern Eigen::Vector3d wrist_products_of_inertia;
extern Eigen::Matrix3d wrist_I_rotated_inertial;
extern Eigen::Vector3d wrist_joint_axis;
extern Eigen::Vector3d wrist_tran_pos_body;

// Gripper link
extern double gripper_link_mass;
extern Eigen::Vector3d gripper_link_com_transformed;
extern Eigen::Vector3d gripper_link_moments_of_inertia;
extern Eigen::Vector3d gripper_link_products_of_inertia;
extern Eigen::Matrix3d gripper_link_I_rotated_inertial;
extern Eigen::Vector3d gripper_link_joint_axis;
extern Eigen::Vector3d gripper_link_tran_pos_body;

// End effector translation
extern Eigen::Vector3d end_effector_trans;

// Function to compute transformations for all links
void computeTransformations();

void computeInertialParams(const Eigen::VectorXd qArm);


#endif