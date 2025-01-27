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

# Automatically get all .cpp files in the src directory
file(GLOB SOURCES src/*.cpp)

# Automatically get all .h files in the include directory
file(GLOB HEADERS include/*.h)

# Manually add main.cpp since it is not in the src folder
list(APPEND SOURCES main.cpp)

# Add the include directory for header files
include_directories(${CMAKE_SOURCE_DIR}/include)


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

