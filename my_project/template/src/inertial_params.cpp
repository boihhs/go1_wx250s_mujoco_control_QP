#include "inertial_params.h"

std::pair<Eigen::Vector3d, Eigen::Matrix3d> inertial_params(const Eigen::VectorXd& qArmInput) {
    

    Eigen::VectorXd qArm(8);
    qArm << 0, 0, qArmInput;

    double robot_mass = 12.03;
    Eigen::Matrix3d robot_MOI;
    robot_MOI << 0.0481, -0.0002, -0.0003,
                -0.0002, 0.2919, -0.0000,
                -0.0003, -0.0000, 0.2887;
    Eigen::Vector3d robot_com = Eigen::Vector3d::Zero();

    std::vector<Eigen::Vector3d> joint_axis = {
        Eigen::Vector3d::Zero(),
        base_joint_axis, shoulder_joint_axis, upper_arm_axis,
        upper_forearm_joint_axis, lower_forearm_joint_axis,
        wrist_joint_axis, gripper_link_joint_axis
    };

    std::vector<Eigen::Vector3d> translations = {
        Eigen::Vector3d::Zero(),
        base_tran_pos_body, shoulder_tran_pos_body, upper_arm_tran_pos_body,
        upper_forearm_tran_pos_body, lower_forearm_tran_pos_body,
        wrist_tran_pos_body, gripper_link_tran_pos_body
    };

    std::vector<double> masses = {
        robot_mass, base_mass, shoulder_mass, upper_arm_mass,
        upper_forearm_mass, lower_forearm_mass, wrist_mass, gripper_link_mass
    };

    std::vector<Eigen::Vector3d> COMs = {
        robot_com, base_com_transformed, shoulder_com_transformed,
        upper_arm_com_transformed, upper_forearm_com_transformed,
        lower_forearm_com_transformed, wrist_com_transformed, gripper_link_com_transformed
    };

    std::vector<Eigen::Matrix3d> MOIs = {
        robot_MOI, base_I_rotated_inertial, shoulder_I_rotated_inertial,
        upper_arm_I_rotated_inertial, upper_forearm_I_rotated_inertial,
        lower_forearm_I_rotated_inertial, wrist_I_rotated_inertial, gripper_link_I_rotated_inertial
    };

    double totalMass = std::accumulate(masses.begin(), masses.end(), 0.0);
    Eigen::Vector3d Moment = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ILumped = Eigen::Matrix3d::Zero();

    Eigen::Matrix4d H_end = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < 8; ++i) {
        // Compute the rotation matrix R using ZYX rotation order
        Eigen::Vector3d axis = joint_axis[i].reverse() * qArm(i);
        Eigen::Matrix3d R = Eigen::AngleAxisd(axis[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(axis[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(axis[2], Eigen::Vector3d::UnitX()).toRotationMatrix();

        // Construct homogeneous transformation matrix H
        Eigen::Matrix4d H;
        H.setIdentity();
        H.block<3, 3>(0, 0) = R;
        H.block<3, 1>(0, 3) = translations[i];  // Set the translation part
        //std::cout << "translations:\n" << joint_axis[i] << std::endl;

        // Update the cumulative transformation matrix
        H_end *= H;

        // Construct H_mid and H
        Eigen::Matrix4d H_mid = Eigen::Matrix4d::Identity();  // Equivalent to [1 0 0; 0 1 0; 0 0 1; 0 0 0] in MATLAB
        Eigen::Vector4d P_mid;
        P_mid << COMs[i].x(), COMs[i].y(), COMs[i].z(), 1.0;  // Append 1 for homogeneous coordinates

        Eigen::Matrix4d H_mid_P;
        H_mid_P.setIdentity();
        H_mid_P.block<4, 1>(0, 3) = P_mid;

        // Compute the global COM transformation
        Eigen::Matrix4d H_COM = H_end * H_mid_P;

        // Extract the transformed COM position
        Eigen::Vector4d P_COM_homogeneous = H_COM * Eigen::Vector4d(0, 0, 0, 1);
        Eigen::Vector3d P_COM = P_COM_homogeneous.head<3>();  // Extract the first 3 elements

        // Accumulate the moment
        Moment += masses[i] * P_COM;

        // Debugging: Print intermediate results
        // std::cout << "H_COM:\n" << H_COM << std::endl;
        // std::cout << "P_COM:\n" << P_COM.transpose() << std::endl;
    }

    

    Eigen::Vector3d COM = Moment / totalMass;

    H_end = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < 8; ++i) {
        // Compute the rotation matrix R using ZYX rotation order
        Eigen::Vector3d axis = joint_axis[i].reverse() * qArm(i);
        Eigen::Matrix3d R = Eigen::AngleAxisd(axis[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(axis[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
                            Eigen::AngleAxisd(axis[2], Eigen::Vector3d::UnitX()).toRotationMatrix();

        // Construct homogeneous transformation matrix H
        Eigen::Matrix4d H;
        H.setIdentity();
        H.block<3, 3>(0, 0) = R;
        H.block<3, 1>(0, 3) = translations[i];  // Set the translation part
        //std::cout << "translations:\n" << joint_axis[i] << std::endl;

        // Update the cumulative transformation matrix
        H_end *= H;

        // Construct H_mid and H
        Eigen::Matrix4d H_mid = Eigen::Matrix4d::Identity();  // Equivalent to [1 0 0; 0 1 0; 0 0 1; 0 0 0] in MATLAB
        Eigen::Vector4d P_mid;
        P_mid << COMs[i].x(), COMs[i].y(), COMs[i].z(), 1.0;  // Append 1 for homogeneous coordinates

        Eigen::Matrix4d H_mid_P;
        H_mid_P.setIdentity();
        H_mid_P.block<4, 1>(0, 3) = P_mid;

        // Compute the global COM transformation
        Eigen::Matrix4d H_COM = H_end * H_mid_P;
 
        Eigen:Matrix3d R_COM = H_COM.block<3, 3>(0, 0);

        // Extract the transformed COM position
        Eigen::Vector4d P_COM_homogeneous = H_COM * Eigen::Vector4d(0, 0, 0, 1);
        Eigen::Vector3d P_COM = P_COM_homogeneous.head<3>() - COM;  // Extract the first 3 elements

        
        Eigen::Matrix3d IWorld_i = R_COM * MOIs[i] * R_COM.transpose();
        Eigen::Matrix3d IOrgin_i = IWorld_i - masses[i] * (skew(P_COM) * skew(P_COM));
        
        ILumped += IOrgin_i;
        //std::cout << "ILumped\n" << MOIs[i] << std::endl;
    }
 

    return {COM, ILumped};
}
