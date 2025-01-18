cmake_minimum_required(VERSION 3.10)
project(TrajProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# Include directories
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# Add sources
set(SOURCES
    utils.cpp
    utils_jac.cpp
    transform_inertial_to_geom.cpp
    Traj_Command_01M.cpp
    Arm_Param.cpp
    inertial_params.cpp
    main.cpp
    MPC_QP.cpp
    transform_inertial_to_geom.cpp
    CartesianControl_01.cpp
    contact_jacobian.cpp
    ContactJacobian_Arm.cpp
    Cartesian_PD_End.cpp
    go1_fk.cpp
)

# Include headers
set(HEADERS
    utils.h
    utils_jac.h
    transform_inertial_to_geom.h
    Traj_Command_01M.h
    Arm_Param.h
    MPC_QP.h
    inertial_params.h
    transform_inertial_to_geom.h
    CartesianControl_01.h
    contact_jacobian.h
    ContactJacobian_Arm.h
    Cartesian_PD_End.h
    go1_fk.h
)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE OsqpEigen::OsqpEigen Eigen3::Eigen)