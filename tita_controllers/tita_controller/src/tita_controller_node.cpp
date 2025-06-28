// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tita_controller/tita_controller_node.hpp"

#include "pluginlib/class_list_macros.hpp"
namespace tita_locomotion
{
TitaController::TitaController() {}

controller_interface::CallbackReturn TitaController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  param_listener_ = std::make_shared<tita_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  setup_controller();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitaController::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }
  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  cmd_vel_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    tita_topic::manager_twist_command, subscribers_qos,
    std::bind(&TitaController::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    tita_topic::manager_pose_command, subscribers_qos,
    std::bind(&TitaController::posestamped_cb, this, std::placeholders::_1));
  fsm_goal_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
    tita_topic::manager_key_command, subscribers_qos,
    std::bind(&TitaController::fsm_goal_cb, this, std::placeholders::_1));
  joy_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
    tita_topic::teleop_command, subscribers_qos,
    std::bind(&TitaController::joy_cb, this, std::placeholders::_1));

  plan_commands_publisher_ = get_node()->create_publisher<locomotion_msgs::msg::PlanCommands>(
    "~/plan_commands", rclcpp::SystemDefaultsQoS());
  robot_states_publisher_ = get_node()->create_publisher<locomotion_msgs::msg::RobotStates>(
    "~/robot_states", rclcpp::SystemDefaultsQoS());
  // rigid_body_subscription_ = get_node()->create_subscription<locomotion_msgs::msg::RigidBody>(
  //   "robot_inertia_calculator/inertia", subscribers_qos,
  //   std::bind(&TitaController::rigid_body_cb, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TitaController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration TitaController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type TitaController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    update_control_parameters();
  }
  controlData_->params->dt = period.seconds();

  wbcTimer_.startTimer();
  // State Update
  int id = 0;
  auto state = controlData_->low_state;
  // clang-format off
  for (std::shared_ptr<Joint> joint : joints_) {
    state->q(id) = joint->position_handle->get().get_value();
    state->dq(id) = joint->velocity_handle->get().get_value();
    state->tau_est(id) = joint->effort_handle->get().get_value();
    id++;
  }
  state->accelerometer = vector3_t(
    imu_sensor_->get_linear_acceleration()[0], 
    imu_sensor_->get_linear_acceleration()[1],
    imu_sensor_->get_linear_acceleration()[2]);
  state->gyro = vector3_t(
    imu_sensor_->get_angular_velocity()[0], 
    imu_sensor_->get_angular_velocity()[1],
    imu_sensor_->get_angular_velocity()[2]);
  state->quat = quaternion_t(
    imu_sensor_->get_orientation()[3], imu_sensor_->get_orientation()[0],
    imu_sensor_->get_orientation()[1], imu_sensor_->get_orientation()[2]);
  // clang-format on
  // Control Update
  if (init_estimator_count_ > 100)
    controlData_->state_estimator->run();
  else
    init_estimator_count_++;
  // controlData_->pinocchio_model->updateState();
  controlData_->state_command->convertFSMState();
  FSMController_->run();
  // Update torque
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->position_command_handle->get().set_value(controlData_->low_cmd->qd[id]);
    joints_[id]->velocity_command_handle->get().set_value(controlData_->low_cmd->qd_dot[id]);
    joints_[id]->effort_command_handle->get().set_value(controlData_->low_cmd->tau_cmd[id]);
    joints_[id]->kp_command_handle->get().set_value(controlData_->low_cmd->kp[id]);
    joints_[id]->kd_command_handle->get().set_value(controlData_->low_cmd->kd[id]);
  }
  wbcTimer_.endTimer();
  // static long int counter = 0;
  // if (counter % 100 == 0) {
  //   RCLCPP_INFO(get_node()->get_logger(), "Last interval for WBC: %f [ms].", mpcTimer_.getLastIntervalInMilliseconds());
  // }
  // counter++;
  // TODO:
  if (!lqr_thread_running_) {
    lqr_thread_running_ = true;
    lqr_thread_ = std::thread(&TitaController::lqr_loop_thread, this);
  }
  plan_commands_cb();
  robot_states_cb();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TitaController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate");
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    const auto position_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    const auto velocity_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);

    // kp command
    const auto kp_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name && interface.get_interface_name() == "kp";
      });
    if (kp_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain kp command handle for %s", joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->kp_command_handle = std::ref(*kp_command_handle);

    // kd command
    const auto kd_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name && interface.get_interface_name() == "kd";
      });
    if (kd_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain kd command handle for %s", joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->kd_command_handle = std::ref(*kd_command_handle);

    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitaController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "########################################################################");
  RCLCPP_INFO(get_node()->get_logger(), "### MPC Benchmarking");
  RCLCPP_INFO(
    get_node()->get_logger(), "###   Maximum : %f [ms].", mpcTimer_.getMaxIntervalInMilliseconds());
  RCLCPP_INFO(
    get_node()->get_logger(), "###   Average : %f [ms].", mpcTimer_.getAverageInMilliseconds());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "########################################################################");
  RCLCPP_INFO(get_node()->get_logger(), "### WBC Benchmarking");
  RCLCPP_INFO(
    get_node()->get_logger(), "###   Maximum : %f [ms].", wbcTimer_.getMaxIntervalInMilliseconds());
  RCLCPP_INFO(
    get_node()->get_logger(), "###   Average : %f [ms].", wbcTimer_.getAverageInMilliseconds());
  release_interfaces();
  imu_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitaController::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitaController::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TitaController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}
// TODO:
void TitaController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{  
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  auto cmd = controlData_->state_command->rc_data_;
  cmd->twist_linear[point::X] = msg->linear.x;
  cmd->twist_linear[point::Y] = msg->linear.y;
  cmd->twist_angular[point::Z] = msg->angular.z;
}

void TitaController::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_stamped_mutex_);
  auto cmd = controlData_->state_command->rc_data_;
  cmd->pose_position[point::Y] = msg->pose.position.y;
  cmd->pose_position[point::Z] = msg->pose.position.z;
  cmd->pose_orientation[quat::QX] = msg->pose.orientation.x;
  cmd->pose_orientation[quat::QY] = msg->pose.orientation.y;
  cmd->pose_orientation[quat::QZ] = msg->pose.orientation.z;
  cmd->pose_orientation[quat::QW] = msg->pose.orientation.w;
}

void TitaController::fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(fsm_goal_mutex_);
  auto cmd = controlData_->state_command->rc_data_;
  cmd->fsm_name_ = msg->data;
}

void TitaController::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joy_mutex_);
  if (joy_msg_ == nullptr) {
    joy_msg_ = std::make_shared<sensor_msgs::msg::Joy>();
    *joy_msg_ = *msg;
  }
  if (msg->axes[4] != joy_msg_->axes[4] || msg->axes[6] != joy_msg_->axes[6]) {
    auto cmd = controlData_->state_command->rc_data_;
    if (msg->axes[4] == 1) {
      cmd->fsm_name_ = "idle";
    } else if (msg->axes[4] == -1 && msg->axes[6] == 1) {
      cmd->fsm_name_ = "transform_up";
    } else if (msg->axes[4] == -1 && msg->axes[6] == -1) {
      cmd->fsm_name_ = "rl";
    }
    *joy_msg_ = std::move(*msg);
  }
}

void TitaController::plan_commands_cb()
{
  locomotion_msgs::msg::PlanCommands msg;

  msg.header.stamp = get_node()->now();
  msg.header.frame_id = "";

  msg.twist_linear.x = controlData_->state_command->desire_data_->twist_linear(point::X);
  msg.twist_linear_int.x = controlData_->state_command->desire_data_->twist_linear_int(point::X);
  msg.twist_linear.y = controlData_->state_command->desire_data_->twist_linear(point::Y);  
  msg.twist_angular.z = controlData_->state_command->desire_data_->twist_angular(point::Z);
  msg.twist_angular_int.z = controlData_->state_command->desire_data_->twist_angular_int(point::Z);

  msg.pose_position.y = controlData_->state_command->desire_data_->pose_position(point::Y);
  msg.pose_position.z = controlData_->state_command->desire_data_->pose_position(point::Z);
  msg.pose_position_dot.y = controlData_->state_command->desire_data_->pose_position_dot(point::Y);
  msg.pose_position_dot.z = controlData_->state_command->desire_data_->pose_position_dot(point::Z);

  msg.pose_rpy.x = controlData_->state_command->desire_data_->pose_rpy(rpy::ROLL);
  msg.pose_rpy.y = controlData_->state_command->desire_data_->pose_rpy(rpy::PITCH);
  msg.pose_rpy.z = controlData_->state_command->desire_data_->pose_rpy(rpy::YAW);
  msg.pose_rpy_dot.x = controlData_->state_command->desire_data_->pose_rpy_dot(rpy::ROLL);
  msg.pose_rpy_dot.y = controlData_->state_command->desire_data_->pose_rpy_dot(rpy::PITCH);
  msg.pose_rpy_dot.z = controlData_->state_command->desire_data_->pose_rpy_dot(rpy::YAW);
  msg.two_wheel_distance = controlData_->state_command->desire_data_->two_wheel_distance;
  msg.two_wheel_distance_dot = controlData_->state_command->desire_data_->two_wheel_distance_dot;
  msg.joint_positions.resize(joints_.size());
  msg.joint_velocities.resize(joints_.size());
  msg.joint_torques.resize(joints_.size());
  for(size_t i = 0; i < joints_.size(); i++){
    msg.joint_positions[i] = controlData_->low_cmd->qd[i];
    msg.joint_velocities[i] = controlData_->low_cmd->qd_dot[i];
    msg.joint_torques[i] = controlData_->low_cmd->tau_cmd[i];
  }
  plan_commands_publisher_->publish(msg);
}

void TitaController::robot_states_cb()
{
  locomotion_msgs::msg::RobotStates msg;

  msg.header.stamp = get_node()->now();
  msg.header.frame_id = "";
  msg.com_position_relative.x = controlData_->wheel_legged_data->com_position_relative(point::X);
  msg.com_position_relative.y = controlData_->wheel_legged_data->com_position_relative(point::Y);
  msg.com_position_relative.z = controlData_->wheel_legged_data->com_position_relative(point::Z);
  msg.com_velocity_relative.x = controlData_->wheel_legged_data->com_velocity_relative(point::X);
  msg.com_velocity_relative.y = controlData_->wheel_legged_data->com_velocity_relative(point::Y);
  msg.com_velocity_relative.z = controlData_->wheel_legged_data->com_velocity_relative(point::Z);

  msg.twist_linear.x = controlData_->wheel_legged_data->twist_linear(point::X);
  msg.twist_linear_int.x = controlData_->wheel_legged_data->twist_linear_int(point::X);
  msg.twist_angular.z = controlData_->wheel_legged_data->twist_angular(point::Z);
  msg.twist_angular_int.z = controlData_->wheel_legged_data->twist_angular_int(point::Z);

  msg.pose_position.y = controlData_->wheel_legged_data->pose_position(point::Y);
  msg.pose_position.z = controlData_->wheel_legged_data->pose_position(point::Z);
  msg.pose_position_dot.y = controlData_->wheel_legged_data->pose_position_dot(point::Y);
  msg.pose_position_dot.z = controlData_->wheel_legged_data->pose_position_dot(point::Z);

  msg.pose_rpy.x = controlData_->wheel_legged_data->pose_rpy(rpy::ROLL);
  msg.pose_rpy.y = controlData_->wheel_legged_data->pose_rpy(rpy::PITCH);
  msg.pose_rpy.z = controlData_->wheel_legged_data->pose_rpy(rpy::YAW);
  msg.pose_rpy_dot.x = controlData_->wheel_legged_data->pose_rpy_dot(rpy::ROLL);
  msg.pose_rpy_dot.y = controlData_->wheel_legged_data->pose_rpy_dot(rpy::PITCH);
  msg.pose_rpy_dot.z = controlData_->wheel_legged_data->pose_rpy_dot(rpy::YAW);
  msg.two_wheel_distance = controlData_->wheel_legged_data->two_wheel_distance;
  msg.two_wheel_distance_dot = controlData_->wheel_legged_data->two_wheel_distance_dot;
  msg.joint_positions.resize(joints_.size());
  msg.joint_velocities.resize(joints_.size());
  msg.joint_torques.resize(joints_.size());
  for(size_t i = 0; i < joints_.size(); i++){
    msg.joint_positions[i] = controlData_->low_state->q[i];
    msg.joint_velocities[i] = controlData_->low_state->dq[i];
    msg.joint_torques[i] = controlData_->low_state->tau_est[i];
  }
  robot_states_publisher_->publish(msg);
}

// void TitaController::rigid_body_cb(const locomotion_msgs::msg::RigidBody::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(rigid_body_mutex_);
//   auto rigid_body = controlData_->pinocchio_model->rigid_body_data_;
//   rigid_body->mass = msg->inertia.m;
//   rigid_body->com(0) = msg->inertia.com.x;
//   rigid_body->com(1) = msg->inertia.com.y;
//   rigid_body->com(2) = msg->inertia.com.z;
//   rigid_body->inertia(0, 0) = msg->inertia.ixx;
//   rigid_body->inertia(1, 1) = msg->inertia.iyy;
//   rigid_body->inertia(2, 2) = msg->inertia.izz;
//   rigid_body->inertia(0, 1) = msg->inertia.ixy;
//   rigid_body->inertia(0, 2) = msg->inertia.ixz;
//   rigid_body->inertia(1, 2) = msg->inertia.iyz;
//   rigid_body->inertia.triangularView<Eigen::StrictlyLower>() =
//     rigid_body->inertia.transpose().triangularView<Eigen::StrictlyLower>();
//   Eigen::MatrixXd diff = rigid_body->inertia - rigid_body->inertia.transpose();
//   if(!diff.isMuchSmallerThan(rigid_body->inertia, 1e-9)){
//     std::cerr << "Inertia matrix is not symmetric!" << std::endl;
//     std::cerr << "Inertia matrix: " << rigid_body->inertia << std::endl;
//   }
//   Eigen::Quaterniond q = Eigen::Quaterniond(
//     msg->transform.orientation.w, msg->transform.orientation.x, msg->transform.orientation.y,
//     msg->transform.orientation.z);

//   rigid_body->transform.rotation() = q.toRotationMatrix();
//   rigid_body->transform.translation() = Eigen::Vector3d(
//     msg->transform.position.x, msg->transform.position.y, msg->transform.position.z);
// }

void TitaController::setup_controller()
{
  controlData_ = std::make_shared<ControlFSMData>(joint_names_.size());
  setup_control_parameters();
  // controlData_->pinocchio_model->setupPinocchioInterface();
  controlData_->state_estimator->addEstimator<VectorNavOrientationEstimator>();
  // controlData_->state_estimator->addEstimator<LinearKFPositionVelocityEstimator>();
  // controlData_->state_command->setup_state_command();

  FSMController_ = std::make_shared<FSM>(controlData_);
}

void TitaController::lqr_loop_thread()
{
  while (lqr_thread_running_) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::high_resolution_clock::now();
    mpcTimer_.startTimer();
    int lqr_update_rate;
    get_node()->get_parameter("lqr_update_rate", lqr_update_rate);

    const std::chrono::duration<scalar_t, std::milli> desired_duration(1000 / lqr_update_rate);
    // controlData_->lqr->update();
    // const auto end_time = std::chrono::high_resolution_clock::now();
    // auto actual_duration =
    //   std::chrono::duration_cast<std::chrono::duration<scalar_t, std::milli>>(end_time - now);
    // RCLCPP_INFO(get_node()->get_logger(), "LQR Desired duration: %f ms, Actual duration: %f ms",
    //             desired_duration.count(), actual_duration.count());
    mpcTimer_.endTimer();
    std::this_thread::sleep_until(
      now +
      std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(desired_duration));
  }
}

void TitaController::setup_control_parameters()
{
  auto param = controlData_->params;
  get_node()->get_parameter<int>("update_rate", param->update_rate);
  get_node()->get_parameter<scalar_t>("wheel_radius", param->wheel_radius);

  get_node()->get_parameter<std::string>("robot_description_path", param->robot_description);

  get_node()->get_parameter<std::vector<std::string>>("wheel_joint_name", param->wheel_joint_name);
  get_node()->get_parameter<std::string>("base_name", param->base_name);
  get_node()->get_parameter<std::string>("ee_name", param->ee_name_);
  get_node()->get_parameter<scalar_t>(
    "static_friction_coefficient", param->static_friction_coefficient);
  get_node()->get_parameter<scalar_t>(
    "sliding_friction_coefficient", param->sliding_friction_coefficient);

  update_control_parameters();

  param->dof_chassis = 8;
  param->dof_arm = 6;
}

void TitaController::update_control_parameters()
{
  auto param = controlData_->params;
  // clang-format off
  get_node()->get_parameter<std::vector<scalar_t>>("torque_limit", param->torque_limit);
  // command limit
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.max_velocity", param->velocity_x_max);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.min_velocity", param->velocity_x_min);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.max_acceleration", param->acceleration_x_max);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.min_acceleration", param->acceleration_x_min);

  get_node()->get_parameter<scalar_t>("task.twist_angular_z.max_velocity", param->velocity_yaw_max);
  get_node()->get_parameter<scalar_t>("task.twist_angular_z.min_velocity", param->velocity_yaw_min);
  get_node()->get_parameter<scalar_t>("task.twist_angular_z.max_acceleration", param->acceleration_yaw_max);
  get_node()->get_parameter<scalar_t>("task.twist_angular_z.min_acceleration", param->acceleration_yaw_min);

  get_node()->get_parameter<scalar_t>("task.pose_orientation_yaw.max_position", param->position_xsplit_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_yaw.max_velocity", param->velocity_xsplit_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_yaw.max_acceleration", param->acceleration_xsplit_max);

  get_node()->get_parameter<scalar_t>("task.pose_orientation_pitch.max_position", param->position_pitch_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_pitch.max_velocity", param->velocity_pitch_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_pitch.max_acceleration", param->acceleration_pitch_max);

  get_node()->get_parameter<scalar_t>("task.pose_orientation_roll.max_position", param->position_roll_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_roll.max_velocity", param->velocity_roll_max);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_roll.max_acceleration", param->acceleration_roll_max);

  get_node()->get_parameter<scalar_t>("task.pose_position_z.max_position", param->position_height_max);
  get_node()->get_parameter<scalar_t>("task.pose_position_z.min_position", param->position_height_min);
  get_node()->get_parameter<scalar_t>("task.pose_position_z.max_velocity", param->velocity_height_max);
  // get_node()->get_parameter<scalar_t>("task.pose_position_z.min_velocity", param->velocity_height_min);
  get_node()->get_parameter<scalar_t>("task.pose_position_z.max_acceleration", param->acceleration_height_max);
  // get_node()->get_parameter<scalar_t>("task.pose_position_z.min_acceleration", param->acceleration_height_min);

  get_node()->get_parameter<scalar_t>("task.pose_position_y.max_position", param->position_ybias_max);
  get_node()->get_parameter<scalar_t>("task.pose_position_y.max_velocity", param->velocity_ybias_max);
  get_node()->get_parameter<scalar_t>("task.pose_position_y.max_acceleration", param->acceleration_ybias_max);

  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.default_value", param->two_wheel_distance);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.max_position", param->position_ysplit_max);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.min_position", param->position_ysplit_min);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.max_velocity", param->velocity_ysplit_max);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.min_velocity", param->velocity_ysplit_min);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.max_acceleration", param->acceleration_ysplit_max);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.min_acceleration", param->acceleration_ysplit_min);

  get_node()->get_parameter<scalar_t>("task.pose_position_y.kp", param->sum_y_kp);
  get_node()->get_parameter<scalar_t>("task.pose_position_z.kp", param->sum_z_kp);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_yaw.kp", param->dif_x_kp);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.kp", param->dif_y_kp);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_roll.kp", param->roll_kp);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_pitch.kp", param->pitch_kp);
  get_node()->get_parameter<scalar_t>("task.twist_angular_z.kp", param->yaw_kp);

  get_node()->get_parameter<scalar_t>("task.pose_position_y.kd", param->sum_y_kd);
  get_node()->get_parameter<scalar_t>("task.pose_position_z.kd", param->sum_z_kd);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_yaw.kd", param->dif_x_kd);
  get_node()->get_parameter<scalar_t>("task.two_wheel_distance.kd", param->dif_y_kd);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_roll.kd", param->roll_kd);
  get_node()->get_parameter<scalar_t>("task.pose_orientation_pitch.kd", param->pitch_kd);
  get_node()->get_parameter<scalar_t>("task.twist_angular_z.kd", param->yaw_kd);

  get_node()->get_parameter<scalar_t>("task.twist_linear_x.q0", param->lqr_q0);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.q1", param->lqr_q1);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.q2", param->lqr_q2);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.q3", param->lqr_q3);
  get_node()->get_parameter<scalar_t>("task.twist_linear_x.r0", param->lqr_r0);

  get_node()->get_parameter<std::vector<scalar_t>>("joint_pd.p", param->joint_kp);
  get_node()->get_parameter<std::vector<scalar_t>>("joint_pd.d", param->joint_kd);

  // get_node()->get_parameter<std::vector<scalar_t>>("wbc_joint_pd.p", param->wbc_joint_kp);
  // get_node()->get_parameter<std::vector<scalar_t>>("wbc_joint_pd.d", param->wbc_joint_kd);
  
  scalar_t kp[4], kd[4];
  get_node()->get_parameter<scalar_t>("single_joint_pd1.p", kp[0]);
  get_node()->get_parameter<scalar_t>("single_joint_pd1.d", kd[0]);
  get_node()->get_parameter<scalar_t>("single_joint_pd2.p", kp[1]);
  get_node()->get_parameter<scalar_t>("single_joint_pd2.d", kd[1]);
  get_node()->get_parameter<scalar_t>("single_joint_pd3.p", kp[2]);
  get_node()->get_parameter<scalar_t>("single_joint_pd3.d", kd[2]);
  get_node()->get_parameter<scalar_t>("single_joint_pd4.p", kp[3]);
  get_node()->get_parameter<scalar_t>("single_joint_pd4.d", kd[3]);
  param->wbc_joint_kp.resize(8);
  param->wbc_joint_kd.resize(8);
  for(size_t i = 0; i < 4; ++i) {
    param->wbc_joint_kp[i] = kp[i];
    param->wbc_joint_kp[i+4] = kp[i];
    param->wbc_joint_kd[i] = kd[i];
    param->wbc_joint_kd[i+4] = kd[i];
  }

  get_node()->get_parameter<std::vector<scalar_t>>("arm_joint_kp", param->arm_joint_kp_);
  get_node()->get_parameter<std::vector<scalar_t>>("arm_joint_kd", param->arm_joint_kd_);
  get_node()->get_parameter<std::vector<scalar_t>>("arm_joint_weight", param->arm_joint_weight_);
  // TODO:
  // get_node()->get_parameter<scalar_t>("bias_sum_y", param->bias_sum_y_);
  // get_node()->get_parameter<scalar_t>("bias_sum_z", param->bias_sum_z_);
  // get_node()->get_parameter<scalar_t>("bias_dif_x", param->bias_dif_x_);
  // get_node()->get_parameter<scalar_t>("bias_dif_y", param->bias_dif_y_);

  get_node()->get_parameter<scalar_t>(
    "estimator.imu_process_noise_position", param->imu_process_noise_position);
  get_node()->get_parameter<scalar_t>(
    "estimator.imu_process_noise_velocity", param->imu_process_noise_velocity);
  get_node()->get_parameter<scalar_t>(
    "estimator.foot_process_noise_position", param->foot_process_noise_position);
  get_node()->get_parameter<scalar_t>(
    "estimator.imu_sensor_noise_position", param->imu_sensor_noise_position);
  get_node()->get_parameter<scalar_t>(
    "estimator.imu_sensor_noise_velocity", param->imu_sensor_noise_velocity);
  get_node()->get_parameter<scalar_t>(
    "estimator.foot_sensor_noise_position", param->foot_sensor_noise_position);
  // clang-format on
  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
}

TitaController::~TitaController()
{
  if (lqr_thread_running_) {
    lqr_thread_running_ = false;
    if (lqr_thread_.joinable()) {
      lqr_thread_.join();
    }
  }
}

}  // namespace tita_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(tita_locomotion::TitaController, controller_interface::ControllerInterface)