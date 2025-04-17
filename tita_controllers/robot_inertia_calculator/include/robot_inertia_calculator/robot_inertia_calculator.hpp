#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "locomotion_msgs/msg/rigid_body.hpp"
#include "tita_utils/topic_names.hpp"
namespace tita_locomotion
{
  Eigen::VectorXd sortJointStates(
      sensor_msgs::msg::JointState::SharedPtr msg, std::vector<std::string> joint_names)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
      if (it != msg->name.end())
      {
        size_t joint_index = std::distance(msg->name.begin(), it);
        q[i] = msg->position[joint_index];
      }
    }
    return q;
  };

  Eigen::Matrix3d skew(Eigen::Vector3d a)
  {
    Eigen::Matrix3d cross_matrix;
    cross_matrix << 0, -a(2), a(1), a(2), 0, -a(0), -a(1), a(0), 0;
    return cross_matrix;
  };

  class ArmConfig
  {
  public:
    ArmConfig(std::shared_ptr<rclcpp::Node> parent_node) : node_(parent_node) {}
    void init(std::string name);
    std::shared_ptr<locomotion_msgs::msg::RigidBody> getInertia();

  private:
    void updateInertia(const sensor_msgs::msg::JointState::SharedPtr msg);
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
    rclcpp::Publisher<locomotion_msgs::msg::RigidBody>::SharedPtr inertia_publisher_;
    std::shared_ptr<locomotion_msgs::msg::RigidBody> inertia_msg_;

  private:
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    std::vector<std::string> joint_names_;
  };

  class RobotMassInertiaCalculator : public rclcpp::Node
  {
  public:
    explicit RobotMassInertiaCalculator(const rclcpp::NodeOptions &options);
    ~RobotMassInertiaCalculator() = default;

  protected:
    void disableCaculate();
    void enableCaculateCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void titaJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void titaImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void updataWholeBodyInertia();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tita_joint_states_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr tita_imu_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr whole_body_com_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_caculate_service_;

    std::shared_ptr<sensor_msgs::msg::JointState> tita_joint_states_msg_;
    std::shared_ptr<sensor_msgs::msg::Imu> tita_imu_msg_;

  private:
    void armModelInit();
    void titaModelInit();
    void diabloPlusProModelInit();
    void addExtraBodyInBase();
    void removeExtraBodyInBase();

    std::vector<std::shared_ptr<ArmConfig>> arm_configs_;
    std::vector<std::string> arm_names_;

    bool enable_calculate_{false};
    std::shared_ptr<pinocchio::Model> tita_model_;
    std::shared_ptr<pinocchio::Data> tita_data_;
    std::vector<std::string> tita_joint_names_;
    pinocchio::Inertia base_saved_inertia_;
  };
} // namespace tita_locomotion