cmake_minimum_required(VERSION 3.5)
project(MR_test)
find_package (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/KDL")

SET(MR include/modern_robotics.cpp)
SET(KDL KDL/PoEKinematics.cpp KDL/PoEKinematics.h KDL/LieDynamics.cpp KDL/LieDynamics.h KDL/LieOperator.cpp KDL/LieOperator.h)

add_executable(main src/main.cpp ${MR} ${KDL})

