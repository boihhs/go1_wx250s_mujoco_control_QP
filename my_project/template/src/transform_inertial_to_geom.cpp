#include "transform_inertial_to_geom.h"

using namespace Eigen;
using namespace std;

// Function to transform inertia from inertial to geometric frame
tuple<Vector3d, Vector3d, Vector3d, Matrix3d> transformInertialToGeom(
    const Vector3d& inertial_pos, const Vector4d& inertial_quat,
    const Matrix3d& diag_inertia, const Vector3d& geom_pos,
    const Vector4d& geom_quat) {
    
    // Compute rotation matrices
    Matrix3d R_geom = quat2rotm(geom_quat/geom_quat.norm());
    Matrix3d R_inertial = quat2rotm(inertial_quat/inertial_quat.norm());


    // Compute center of mass in geometric frame
    Vector3d com_geom = R_geom.transpose() * (inertial_pos - geom_pos);

    // Compute rotated inertia
    Matrix3d I_rotated = R_geom.transpose() * R_inertial.transpose() * diag_inertia * R_inertial * R_geom;
    Matrix3d I_rotated_inertial = R_inertial.transpose() * diag_inertia * R_inertial;

    // Compute moments and products of inertia
    Vector3d I_moments = I_rotated.diagonal();
    Vector3d I_products(I_rotated(1, 2), I_rotated(0, 2), I_rotated(0, 1));

    //std::cout << "\n" << diag_inertia << std::endl;

    // Return all outputs as a tuple
    return make_tuple(com_geom, I_moments, I_products, I_rotated_inertial);
}
