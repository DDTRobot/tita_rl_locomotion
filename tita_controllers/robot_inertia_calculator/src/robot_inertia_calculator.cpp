#include "robot_inertia_calculator/robot_inertia_calculator.hpp"
namespace tita_locomotion
{
  void ArmConfig::init(std::string arm_name)
  {
    RCLCPP_INFO(node_->get_logger(), "Arm name is %s", arm_name.c_str());

    model_ = std::make_unique<pinocchio::Model>();
    std::string urdf_path;
    node_->get_parameter<std::string>(arm_name + ".urdf_path", urdf_path);
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);

    for (auto name : model_->names)
    {
      if (name != "universe" && name != "root_joint")
      {
        joint_names_.push_back(name);
      }
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    std::string joint_states_name = "/joint_states";
    node_->get_parameter<std::string>(arm_name + ".joint_states_name", joint_states_name);

    joint_states_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_name, subscribers_qos,
        std::bind(&ArmConfig::updateInertia, this, std::placeholders::_1));

    inertia_publisher_ = node_->create_publisher<locomotion_msgs::msg::RigidBody>(
        "robot_inertia_calculator/inertia/" + arm_name, 1);

    inertia_msg_ = std::make_shared<locomotion_msgs::msg::RigidBody>();
    double position_x{0}, position_y{0}, position_z{0};
    double orientation_w{1}, orientation_x{0}, orientation_y{0}, orientation_z{0};
    node_->get_parameter<double>(arm_name + ".transform.position.x", position_x);
    node_->get_parameter<double>(arm_name + ".transform.position.y", position_y);
    node_->get_parameter<double>(arm_name + ".transform.position.z", position_z);
    node_->get_parameter<double>(arm_name + ".transform.orientation.w", orientation_w);
    node_->get_parameter<double>(arm_name + ".transform.orientation.x", orientation_x);
    node_->get_parameter<double>(arm_name + ".transform.orientation.y", orientation_y);
    node_->get_parameter<double>(arm_name + ".transform.orientation.z", orientation_z);
    inertia_msg_->transform.position.x = position_x;
    inertia_msg_->transform.position.y = position_y;
    inertia_msg_->transform.position.z = position_z;
    inertia_msg_->transform.orientation.w = orientation_w;
    inertia_msg_->transform.orientation.x = orientation_x;
    inertia_msg_->transform.orientation.y = orientation_y;
    inertia_msg_->transform.orientation.z = orientation_z;
  }

  void ArmConfig::updateInertia(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
    q(6) = 1;

    q.tail(joint_names_.size()) = sortJointStates(msg, joint_names_);

    pinocchio::centerOfMass(*model_, *data_, q);
    Eigen::Vector3d com = data_->com[0]; // 整体质心在基座坐标系下的位置
    Eigen::Matrix3d inertia_base = Eigen::Matrix3d::Zero();
    double total_mass = 0.0;
    for (int i = 0; i < model_->njoints; ++i)
    {
      // 跳过虚拟关节（如根节点）
      if (model_->inertias[i].mass() == 0)
        continue;

      // 获取第i个链接的质心位置和惯性矩阵
      pinocchio::SE3 oMi = data_->oMi[i];
      Eigen::Matrix3d r = oMi.rotation(); // 局部质心到基座的旋转矩阵
      Eigen::Vector3d c_i =
          oMi.translation() + r * model_->inertias[i].lever(); // 质心在基座下的位置
      double m_i = model_->inertias[i].mass();
      Eigen::Matrix3d inertia_ci = model_->inertias[i].inertia();      // 局部惯性矩阵
      Eigen::Matrix3d inertia_cibase = r * inertia_ci * r.transpose(); // 转移到base

      // 计算链接的惯性矩阵在基座坐标系下
      Eigen::Matrix3d inertia_i =
          inertia_cibase +
          m_i * (c_i.transpose() * c_i * Eigen::Matrix3d::Identity() - c_i * c_i.transpose());
      inertia_base += inertia_i;
      total_mass += m_i;
    }
    Eigen::Matrix3d inertia_total =
        inertia_base -
        total_mass * (com.transpose() * com * Eigen::Matrix3d::Identity() - com * com.transpose());

    inertia_msg_->inertia.m = total_mass;
    inertia_msg_->inertia.com.x = com[0];
    inertia_msg_->inertia.com.y = com[1];
    inertia_msg_->inertia.com.z = com[2];
    inertia_msg_->inertia.ixx = inertia_total(0, 0);
    inertia_msg_->inertia.ixy = inertia_total(0, 1);
    inertia_msg_->inertia.ixz = inertia_total(0, 2);
    inertia_msg_->inertia.iyy = inertia_total(1, 1);
    inertia_msg_->inertia.iyz = inertia_total(1, 2);
    inertia_msg_->inertia.izz = inertia_total(2, 2);

    inertia_publisher_->publish(*inertia_msg_);
  }

  std::shared_ptr<locomotion_msgs::msg::RigidBody> ArmConfig::getInertia() { return inertia_msg_; }

  RobotMassInertiaCalculator::RobotMassInertiaCalculator(const rclcpp::NodeOptions &options)
      : Node("robot_inertia_calculator", options)
  {
    enable_caculate_service_ = this->create_service<std_srvs::srv::SetBool>(
        "robot_inertia_calculator/enable_calculate",
        std::bind(
            &RobotMassInertiaCalculator::enableCaculateCallback, this, std::placeholders::_1,
            std::placeholders::_2));
  }

  void RobotMassInertiaCalculator::disableCaculate()
  {
    arm_configs_.clear();
    arm_configs_.shrink_to_fit();

    whole_body_com_publisher_.reset();
    tilt_publisher_.reset();

    tita_joint_states_subscription_.reset();
    tita_imu_subscription_.reset();

    tita_model_.reset();
    tita_data_.reset();

    tita_joint_states_msg_.reset();
    tita_imu_msg_.reset();

    tita_joint_names_.clear();
    tita_joint_names_.shrink_to_fit();

    base_saved_inertia_ = pinocchio::Inertia(); // 根据需要重置为默认的惯性值
  }

  void RobotMassInertiaCalculator::enableCaculateCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data && !enable_calculate_)
    {
      enable_calculate_ = true;
      response->success = true;
      response->message = "Inertia caculate Enabled.";
      armModelInit();
      titaModelInit();
      // diabloPlusProModelInit();
    }
    else if (!request->data && enable_calculate_)
    {
      enable_calculate_ = false;
      response->success = true;
      response->message = "Inertia caculate Disabled.";
      disableCaculate();
    }
    else
    {
      response->success = false;
      response->message = "Inertia caculate already in this state.";
    }

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void RobotMassInertiaCalculator::armModelInit()
  {
    this->get_parameter<std::vector<std::string>>("arm_names", arm_names_);
    for (auto name : arm_names_)
    {
      auto arm = std::make_shared<ArmConfig>(shared_from_this());
      arm->init(name);
      arm_configs_.push_back(arm);
    }
  }

  void RobotMassInertiaCalculator::titaModelInit()
  {
    tita_model_ = std::make_shared<pinocchio::Model>();
    std::string tita_urdf_path;
    this->get_parameter<std::string>("tita_urdf_path", tita_urdf_path);
    pinocchio::urdf::buildModel(tita_urdf_path, pinocchio::JointModelFreeFlyer(), *tita_model_);
    tita_data_ = std::make_shared<pinocchio::Data>(*tita_model_);
    for (auto name : tita_model_->names)
    {
      if (name != "universe" && name != "root_joint")
      {
        tita_joint_names_.push_back(name);
      }
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    tita_joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        tita_topic::joint_states, subscribers_qos,
        std::bind(&RobotMassInertiaCalculator::titaJointsCallback, this, std::placeholders::_1));

    tita_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        tita_topic::body_imu, subscribers_qos,
        std::bind(&RobotMassInertiaCalculator::titaImuCallback, this, std::placeholders::_1));

    whole_body_com_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "robot_inertia_calculator/whole_body_com", rclcpp::SystemDefaultsQoS());
    tilt_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "robot_inertia_calculator/set_tilt", rclcpp::SystemDefaultsQoS());

    tita_joint_states_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
    tita_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();
    base_saved_inertia_ =
        tita_model_->inertias[tita_model_->frames[tita_model_->getFrameId("base_link")].parent];
  }

  void RobotMassInertiaCalculator::diabloPlusProModelInit()
  {
    tita_model_ = std::make_shared<pinocchio::Model>();
    std::string diablo_pluspro_urdf_path;
    this->get_parameter<std::string>("diablo_pluspro_urdf_path", diablo_pluspro_urdf_path);
    pinocchio::urdf::buildModel(diablo_pluspro_urdf_path, pinocchio::JointModelFreeFlyer(), *tita_model_);
    tita_data_ = std::make_shared<pinocchio::Data>(*tita_model_);
    for (auto name : tita_model_->names)
    {
      if (name != "universe" && name != "root_joint")
      {
        tita_joint_names_.push_back(name);
      }
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    tita_joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        tita_topic::joint_states, subscribers_qos,
        std::bind(&RobotMassInertiaCalculator::titaJointsCallback, this, std::placeholders::_1));

    tita_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        tita_topic::body_imu, subscribers_qos,
        std::bind(&RobotMassInertiaCalculator::titaImuCallback, this, std::placeholders::_1));

    whole_body_com_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "robot_inertia_calculator/whole_body_com", rclcpp::SystemDefaultsQoS());
    tilt_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "robot_inertia_calculator/set_tilt", rclcpp::SystemDefaultsQoS());

    tita_joint_states_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
    tita_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();
    base_saved_inertia_ =
        tita_model_->inertias[tita_model_->frames[tita_model_->getFrameId("base_link")].parent];
  }

  void RobotMassInertiaCalculator::titaJointsCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    *tita_joint_states_msg_ = std::move(*msg);
  }

  void RobotMassInertiaCalculator::titaImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    *tita_imu_msg_ = std::move(*msg);
    updataWholeBodyInertia();
  }

  void RobotMassInertiaCalculator::updataWholeBodyInertia()
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(tita_model_->nq);
    tf2::Quaternion quat(
        tita_imu_msg_->orientation.x, tita_imu_msg_->orientation.y, tita_imu_msg_->orientation.z,
        tita_imu_msg_->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tf2::Quaternion quat_modified;
    quat_modified.setRPY(roll, pitch, 0);
    for (auto i = 0; i < 4; i++)
    {
      q(i + 3) = quat_modified[i];
    }

    q.tail(tita_joint_names_.size()) = sortJointStates(tita_joint_states_msg_, tita_joint_names_);
    addExtraBodyInBase();
    pinocchio::centerOfMass(*tita_model_, *tita_data_, q);
    removeExtraBodyInBase();
    Eigen::Vector3d com = tita_data_->com[0];
    auto msg = std::make_shared<geometry_msgs::msg::Vector3>();
    msg->x = com[0];
    msg->y = com[1];
    msg->z = com[2];
    whole_body_com_publisher_->publish(*msg);

    pinocchio::updateFramePlacements(*tita_model_, *tita_data_);
    auto left_pos =
        tita_data_->oMf[tita_model_->getFrameId(tita_joint_names_[tita_joint_names_.size() / 2 - 1])]
            .translation();
    auto right_pos =
        tita_data_->oMf[tita_model_->getFrameId(tita_joint_names_[tita_joint_names_.size() - 1])]
            .translation();
    auto height = -(left_pos[2] + right_pos[2]) / 2;
    if (height < 0.01)
    {
      height = 0.01; // TODO: why?
    }
    auto tilt = std::atan2(-com[0], height);
    auto tilt_msg = std::make_shared<std_msgs::msg::Float64>();
    tilt_msg->data = tilt;
    tilt_publisher_->publish(*tilt_msg);
  }

  void RobotMassInertiaCalculator::addExtraBodyInBase()
  {
    for (auto arm : arm_configs_)
    {
      auto msg = arm->getInertia();
      double mass = msg->inertia.m;

      Eigen::Vector3d com = Eigen::Vector3d(
          msg->inertia.com.x, msg->inertia.com.y, msg->inertia.com.z);

      Eigen::Matrix3d inertia;
      inertia << msg->inertia.ixx, msg->inertia.ixy, msg->inertia.ixz,
          msg->inertia.ixy, msg->inertia.iyy, msg->inertia.iyz,
          msg->inertia.ixz, msg->inertia.iyz, msg->inertia.izz;

      pinocchio::SE3 transform;

      Eigen::Quaterniond q = Eigen::Quaterniond(
          msg->transform.orientation.w, msg->transform.orientation.x,
          msg->transform.orientation.y, msg->transform.orientation.z);
      transform.rotation() = q.toRotationMatrix();

      transform.translation() = Eigen::Vector3d(
          msg->transform.position.x, msg->transform.position.y,
          msg->transform.position.z);

      pinocchio::Inertia inertia_msg(mass, com, inertia);
      pinocchio::JointIndex base_joint_id =
          tita_model_->frames[tita_model_->getFrameId("base_link")].parent;
      tita_model_->appendBodyToJoint(base_joint_id, inertia_msg, transform);
    }
  }

  void RobotMassInertiaCalculator::removeExtraBodyInBase()
  {
    pinocchio::JointIndex base_joint_id =
        tita_model_->frames[tita_model_->getFrameId("base_link")].parent;
    tita_model_->inertias[base_joint_id] = base_saved_inertia_;
  }

} // namespace tita_locomotion