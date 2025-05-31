/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some controlParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H
#include <string>
#include <vector>

#include "common/cppTypes.h"

struct RobotControlParameters
{
  RobotControlParameters(){
    torque_limit = {80.0, 20.0, 40.0, 5.0, 80.0, 20.0, 40.0, 5.0};
    joint_kp = {600.0, 80.0, 80.0, 5.0, 600.0, 80.0, 80.0, 5.0};
    joint_kd = {8.0, 8.0, 8.0, 0.2, 8.0, 8.0, 8.0, 0.2};
    rl_joint_kp = {600.0, 80.0, 80.0, 5.0, 600.0, 80.0, 80.0, 5.0};  //rl mode parameters
    rl_joint_kd = {8.0, 8.0, 8.0, 0.2, 8.0, 8.0, 8.0, 0.2};
    wheel_joint_name = {"joint_left_leg_4", "joint_right_leg_4"};
    
  }
  scalar_t wheel_radius{0.0925};
  scalar_t sliding_friction_coefficient{0.5};
  scalar_t static_friction_coefficient{0.8};
  std::string base_name{"base_link"};
  std::vector<std::string> wheel_joint_name;
  std::string robot_description{"/usr/share/robot_description/tita/urdf/robot.urdf"};
  size_t dof_chassis{8}, dof_arm{6};

  // control param
  scalar_t dt;           // actual control period
  int update_rate{500};  // control loop update rate
  std::vector<scalar_t> torque_limit;
  std::vector<scalar_t> joint_kp, joint_kd;
  std::vector<scalar_t> rl_joint_kp, rl_joint_kd;  //pd parameters for rl
  std::vector<scalar_t> wbc_joint_kp, wbc_joint_kd;

  scalar_t sum_y_kp{200}, sum_y_kd{10};
  scalar_t sum_z_kp{200}, sum_z_kd{10};
  scalar_t dif_x_kp{200}, dif_x_kd{10};
  scalar_t dif_y_kp{200}, dif_y_kd{10};

  scalar_t roll_kp{200}, roll_kd{10};
  scalar_t pitch_kp{200}, pitch_kd{10};
  scalar_t yaw_kp{200}, yaw_kd{10};

  scalar_t lqr_q0{60}, lqr_q1{800}, lqr_q2{6800}, lqr_q3{1500}, lqr_r0{1};

  // command limit
  scalar_t velocity_x_max{+1.0}, acceleration_x_max{+3.0};
  scalar_t velocity_x_min{-1.0}, acceleration_x_min{-3.0};

  scalar_t velocity_yaw_max{+1.0}, acceleration_yaw_max{+3.0};
  scalar_t velocity_yaw_min{-1.0}, acceleration_yaw_min{-3.0};

  scalar_t position_roll_max{0.2}, velocity_roll_max{0.8}, acceleration_roll_max{30.0};

  scalar_t position_pitch_max{0.2}, velocity_pitch_max{0.8}, acceleration_pitch_max{30.0};

  scalar_t position_height_max{0.4}, velocity_height_max{+0.2}, acceleration_height_max{+30.0};
  scalar_t position_height_min{0.2}, velocity_height_min{-0.2}, acceleration_height_min{-30.0};

  scalar_t position_xsplit_max{0.05}, velocity_xsplit_max{0.2}, acceleration_xsplit_max{30.0};
  scalar_t two_wheel_distance{0.2835};
  scalar_t position_ysplit_max{+0.02}, velocity_ysplit_max{+0.2},
    acceleration_ysplit_max{+30.0};  // dif_y
  scalar_t position_ysplit_min{-0.02}, velocity_ysplit_min{-0.2}, acceleration_ysplit_min{-30.0};

  scalar_t position_ybias_max{+0.02}, velocity_ybias_max{+0.2},
    acceleration_ybias_max{+30.0};  // sum_y

  // compensation param
  scalar_t bias_sum_y_{0}, bias_sum_z_{0}, bias_dif_x_{0}, bias_dif_y_{0};
  // state estimator param
  scalar_t imu_process_noise_position{0.02};
  scalar_t imu_process_noise_velocity{0.02};
  scalar_t foot_process_noise_position{0.002};

  scalar_t imu_sensor_noise_position{0.005};
  scalar_t imu_sensor_noise_velocity{0.1};
  scalar_t foot_sensor_noise_position{0.01};
  // manupalator param
  std::string ee_name_;
  std::vector<scalar_t> arm_joint_kp_, arm_joint_kd_, arm_joint_weight_;
};

#endif  // PROJECT_ROBOTPARAMETERS_H
