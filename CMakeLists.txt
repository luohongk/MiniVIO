cmake_minimum_required(VERSION 3.10)
project(MiniVIO)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Debug, Release, RelWithDebInfo")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -DDEBUG")

# Output to build/
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")

# Include paths
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/thirdparty/camodocal/include
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# ============================================================
#  Camera Model Library (thirdparty)
# ============================================================
add_library(camera_model SHARED
    thirdparty/camodocal/src/camera_models/chessboard/Chessboard.cc
    thirdparty/camodocal/src/camera_models/calib/CameraCalibration.cc
    thirdparty/camodocal/src/camera_models/camera_models/Camera.cc
    thirdparty/camodocal/src/camera_models/camera_models/CameraFactory.cc
    thirdparty/camodocal/src/camera_models/camera_models/CostFunctionFactory.cc
    thirdparty/camodocal/src/camera_models/camera_models/PinholeCamera.cc
    thirdparty/camodocal/src/camera_models/camera_models/CataCamera.cc
    thirdparty/camodocal/src/camera_models/camera_models/EquidistantCamera.cc
    thirdparty/camodocal/src/camera_models/camera_models/ScaramuzzaCamera.cc
    thirdparty/camodocal/src/camera_models/sparse_graph/Transform.cc
    thirdparty/camodocal/src/camera_models/gpl/gpl.cc
    thirdparty/camodocal/src/camera_models/gpl/EigenQuaternionParameterization.cc
)
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# ============================================================
#  MiniVIO Core Library
# ============================================================
add_library(MiniVIO SHARED
    # System
    src/system/system.cpp
    src/system/parameters.cpp
    src/system/estimator.cpp
    # Frontend
    src/frontend/feature_tracker.cpp
    src/frontend/feature_manager.cpp
    # Initial
    src/initial/solve_5pts.cpp
    src/initial/initial_alignment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    # Backend
    src/backend/vertex.cc
    src/backend/edge.cc
    src/backend/problem.cc
    src/backend/vertex_pose.cc
    src/backend/edge_reprojection.cc
    src/backend/edge_imu.cc
    src/backend/edge_prior.cpp
    src/backend/loss_function.cc
    src/backend/imu_integration.cc
    # Utility
    src/utility/utility.cpp
)
target_link_libraries(MiniVIO
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${Pangolin_LIBRARIES}
    camera_model
)

# ============================================================
#  Executables
# ============================================================
add_executable(run_euroc app/run_euroc.cpp)
target_link_libraries(run_euroc MiniVIO -lpthread)

add_executable(curve_fitting app/curve_fitting.cpp)
target_link_libraries(curve_fitting MiniVIO)
