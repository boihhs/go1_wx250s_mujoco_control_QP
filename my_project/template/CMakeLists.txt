cmake_minimum_required(VERSION 3.10)
project(TrajProject)

# ------------------------------------------------------------------------------
# 1) C++ standard and build type
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)   # or Release, as desired

# ------------------------------------------------------------------------------
# 2) Include / Link directories
#    We replicate the effect of -I../../lib and -L../../lib from your old Makefile
# ------------------------------------------------------------------------------
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../include")

# ------------------------------------------------------------------------------
# 3) Compiler and linker flags (from your old Makefile's COMMON)
#    -O2, -pthread, -Wl,-no-as-needed, and RPATH to local libraries
# ------------------------------------------------------------------------------
add_compile_options(-O2)
set(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -pthread -Wl,-no-as-needed -Wl,-rpath,'\$ORIGIN'/../../lib"
)

# ------------------------------------------------------------------------------
# 4) Dependencies via find_package (if installed in a standard location)
# ------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# ------------------------------------------------------------------------------
# 5) Source files
#    (The headers are optional in add_executable, but included for clarity)
# ------------------------------------------------------------------------------
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

# ------------------------------------------------------------------------------
# 6) Create the executable
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# ------------------------------------------------------------------------------
# 7) Link the libraries (MuJoCo, GLFW, OsqpEigen, Eigen, etc.)
# ------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        mujoco
        glfw
        OsqpEigen::OsqpEigen
        Eigen3::Eigen
)

