#ifndef TRANSFORM_INERTIA_H
#define TRANSFORM_INERTIA_H

#include <Eigen/Dense>
#include <tuple> // For returning multiple values
#include "utils.h"

using namespace Eigen;
using namespace std;

// Function to transform inertia from inertial to geometric frame
tuple<Vector3d, Vector3d, Vector3d, Matrix3d> transformInertialToGeom(
    const Vector3d& inertial_pos, const Vector4d& inertial_quat,
    const Matrix3d& diag_inertia, const Vector3d& geom_pos,
    const Vector4d& geom_quat);

#endif // TRANSFORM_INERTIA_H
