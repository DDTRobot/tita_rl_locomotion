cmake_minimum_required(VERSION 3.8)
project(tita_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(CUDA REQUIRED)

include_directories()
include_directories(/usr/local/cuda-12.1/include)
include_directories((/usr/src/tensorrt-8.6/include))
link_directories(/usr/local/cuda-12.1/lib64)
link_directories(/usr/src/tensorrt-8.6/lib)
# generate parameter listener hpp file
generate_parameter_library(tita_controller_parameters
  src/tita_controller_parameters.yaml
)

pluginlib_export_plugin_description_file(controller_interface tita_controller_plugin.xml)

set(ESTIMATOR_FILES
  src/estimator/OrientationEstimator.cpp
)
set(FSM_FILES
  src/fsm/FSM.cpp
  src/fsm/FSMState.cpp
  src/fsm/FSMState_JointPD.cpp
  src/fsm/FSMState_Passive.cpp
  src/fsm/FSMState_RecoveryStand.cpp
  src/fsm/FSMState_TransformDown.cpp
  src/fsm/FSMState_RL.cpp
)

set(PLANNER_FILES
  src/planner/DesiredStateCommand.cpp
)

set(TENSORRT_CUDA_FILES
  src/tensorrt_cuda/tensor_cuda_test.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${ESTIMATOR_FILES} ${FSM_FILES} ${LQR_FILES} ${PLANNER_FILES} ${WBC_FILES} ${TENSORRT_CUDA_FILES}
  src/tita_controller_node.cpp
)
if(DEBUG)
  target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g)
endif()

target_link_libraries(tita_controller tita_controller_parameters cudart nvinfer)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


endif()

ament_auto_package()
