cmake_minimum_required(VERSION 3.0.2)
project(control_demo)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)






catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_demo
#  CATKIN_DEPENDS roscpp
  DEPENDS gflags
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

set(acado_mpc_dep_src
  ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_integrator.c
  ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_solver.c
  ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_auxiliary_functions.c
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/Bounds.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/Constraints.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/CyclingManager.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/Indexlist.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/MessageHandling.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/QProblem.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/QProblemB.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/SubjectTo.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/Utils.cpp 
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp)

add_library(acado_mpc 
  STATIC
  src/acado_mpc/mpc_controller.cpp
  src/acado_mpc/mpc_wrapper.cpp
  src/acado_mpc/mpc_common.cpp
  ${acado_mpc_dep_src})

target_include_directories(acado_mpc PUBLIC
  ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen
  ${PROJECT_SOURCE_DIR}/external/qpoases
  ${PROJECT_SOURCE_DIR}/external/qpoases/INCLUDE
  ${PROJECT_SOURCE_DIR}/external/qpoases/SRC)

target_link_libraries(acado_mpc 
  ${catkin_LIBRARIES}
  Eigen3::Eigen)

add_library(mpc_base
  STATIC
  src/control_demo/MpcBase.cpp)
target_link_libraries(mpc_base
  acado_mpc
  Eigen3::Eigen
  ${catkin_LIBRARIES})


add_executable(node 
  src/node.cpp)
target_link_libraries(node
  mpc_base
  Eigen3::Eigen
  gflags
  ${catkin_LIBRARIES})




