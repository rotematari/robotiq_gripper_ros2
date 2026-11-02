// RViz panel for robotiq_gripper_driver: go to a width with a speed and force,
// open, close, stop, activate and reset, with a live status line.

#ifndef ROBOTIQ_GRIPPER_RVIZ__GRIPPER_PANEL_HPP_
#define ROBOTIQ_GRIPPER_RVIZ__GRIPPER_PANEL_HPP_

#include <memory>
#include <string>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotiq_gripper_msgs/msg/gripper_status.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;

namespace robotiq_gripper_rviz
{

class GripperPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GripperPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void connectToDriver();
  void sendGoal(double width_mm);
  void callTrigger(const QString & name);
  void showStatus();
  void showMessage(const QString & text);

private:
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using Status = robotiq_gripper_msgs::msg::GripperStatus;

  void onStatus(Status::ConstSharedPtr msg);
  void applyRanges(const Status & msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr activate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

  Status latest_;
  bool have_status_ = false;
  double stroke_mm_ = 0.0;

  QLineEdit * namespace_edit_;
  QLabel * status_label_;
  QLabel * message_label_;
  QSlider * width_slider_;
  QDoubleSpinBox * width_spin_;
  QDoubleSpinBox * speed_spin_;
  QDoubleSpinBox * force_spin_;
};

}  // namespace robotiq_gripper_rviz

#endif  // ROBOTIQ_GRIPPER_RVIZ__GRIPPER_PANEL_HPP_
