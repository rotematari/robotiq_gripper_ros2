#include "robotiq_gripper_rviz/gripper_panel.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace robotiq_gripper_rviz
{

namespace
{
// The slider works in steps of 0.1 mm.
constexpr int kSliderSteps = 10;

QDoubleSpinBox * makeSpin(const QString & suffix, int decimals)
{
  auto * spin = new QDoubleSpinBox;
  spin->setSuffix(suffix);
  spin->setDecimals(decimals);
  spin->setKeyboardTracking(false);
  return spin;
}
}  // namespace

GripperPanel::GripperPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  namespace_edit_ = new QLineEdit("/robotiq_gripper");
  namespace_edit_->setToolTip("Namespace of the robotiq_gripper node (its ~/ interfaces)");
  status_label_ = new QLabel("Waiting for status...");
  status_label_->setWordWrap(true);
  message_label_ = new QLabel;
  message_label_->setWordWrap(true);

  width_slider_ = new QSlider(Qt::Horizontal);
  width_spin_ = makeSpin(" mm", 1);
  speed_spin_ = makeSpin(" mm/s", 0);
  force_spin_ = makeSpin(" N", 0);
  force_spin_->setToolTip(
    "Gripper force limit. On contact the fingers stop and hold with about this force "
    "(approximate scale; the lowest setting also disables automatic re-grasp).");

  auto * activate = new QPushButton("Activate");
  activate->setToolTip("Activation moves the fingers through their full stroke");
  auto * open = new QPushButton("Open");
  auto * close = new QPushButton("Close");
  auto * go = new QPushButton("Go to width");
  auto * stop = new QPushButton("Stop");
  auto * reset = new QPushButton("Reset");
  reset->setToolTip("Deactivate and clear faults; activate again before moving");

  auto * ns_row = new QHBoxLayout;
  ns_row->addWidget(new QLabel("Driver:"));
  ns_row->addWidget(namespace_edit_);

  auto * grid = new QGridLayout;
  grid->addWidget(new QLabel("Width"), 0, 0);
  grid->addWidget(width_slider_, 0, 1);
  grid->addWidget(width_spin_, 0, 2);
  grid->addWidget(new QLabel("Speed"), 1, 0);
  grid->addWidget(speed_spin_, 1, 2);
  grid->addWidget(new QLabel("Force"), 2, 0);
  grid->addWidget(force_spin_, 2, 2);

  auto * buttons = new QGridLayout;
  buttons->addWidget(go, 0, 0, 1, 3);
  buttons->addWidget(open, 1, 0);
  buttons->addWidget(close, 1, 1);
  buttons->addWidget(stop, 1, 2);
  buttons->addWidget(activate, 2, 0);
  buttons->addWidget(reset, 2, 2);

  auto * layout = new QVBoxLayout;
  layout->addLayout(ns_row);
  layout->addWidget(status_label_);
  layout->addLayout(grid);
  layout->addLayout(buttons);
  layout->addWidget(message_label_);
  layout->addStretch();
  setLayout(layout);

  connect(width_slider_, &QSlider::valueChanged, this, [this](int value) {
      width_spin_->blockSignals(true);
      width_spin_->setValue(value / static_cast<double>(kSliderSteps));
      width_spin_->blockSignals(false);
    });
  connect(
    width_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v) {
      width_slider_->blockSignals(true);
      width_slider_->setValue(static_cast<int>(std::lround(v * kSliderSteps)));
      width_slider_->blockSignals(false);
    });
  connect(namespace_edit_, &QLineEdit::editingFinished, this, &GripperPanel::connectToDriver);
  connect(go, &QPushButton::clicked, this, [this]() {sendGoal(width_spin_->value());});
  connect(open, &QPushButton::clicked, this, [this]() {sendGoal(stroke_mm_);});
  connect(close, &QPushButton::clicked, this, [this]() {sendGoal(0.0);});
  connect(stop, &QPushButton::clicked, this, [this]() {callTrigger("stop");});
  connect(activate, &QPushButton::clicked, this, [this]() {callTrigger("activate");});
  connect(reset, &QPushButton::clicked, this, [this]() {callTrigger("reset");});
}

void GripperPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectToDriver();
}

void GripperPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", namespace_edit_->text());
}

void GripperPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("Namespace", &ns)) {
    namespace_edit_->setText(ns);
    if (node_) {
      connectToDriver();
    }
  }
}

void GripperPanel::connectToDriver()
{
  if (!node_) {
    return;
  }
  std::string ns = namespace_edit_->text().trimmed().toStdString();
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }
  have_status_ = false;
  status_label_->setText("Waiting for status...");
  status_sub_ = node_->create_subscription<Status>(
    ns + "/status", 10, [this](Status::ConstSharedPtr msg) {onStatus(msg);});
  action_client_ = rclcpp_action::create_client<GripperCommand>(node_, ns + "/gripper_cmd");
  activate_client_ = node_->create_client<std_srvs::srv::Trigger>(ns + "/activate");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>(ns + "/stop");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>(ns + "/reset");
  Q_EMIT configChanged();
}

void GripperPanel::onStatus(Status::ConstSharedPtr msg)
{
  // ROS callbacks may run outside the Qt thread; hand the message over.
  QPointer<GripperPanel> self(this);
  QMetaObject::invokeMethod(
    this, [self, this, msg]() {
      if (!self) {
        return;
      }
      const bool ranges_changed = !have_status_ || msg->model != latest_.model;
      latest_ = *msg;
      have_status_ = true;
      if (ranges_changed) {
        applyRanges(latest_);
      }
      showStatus();
    }, Qt::QueuedConnection);
}

void GripperPanel::applyRanges(const Status & msg)
{
  stroke_mm_ = msg.stroke * 1000.0;
  width_slider_->setRange(0, static_cast<int>(std::lround(stroke_mm_ * kSliderSteps)));
  width_spin_->setRange(0.0, stroke_mm_);
  width_spin_->setValue(msg.width * 1000.0);
  speed_spin_->setRange(msg.min_speed * 1000.0, msg.max_speed * 1000.0);
  speed_spin_->setValue(msg.min_speed * 1000.0 + 0.25 * (msg.max_speed - msg.min_speed) * 1000.0);
  force_spin_->setRange(msg.min_force, msg.max_force);
  force_spin_->setValue(msg.min_force);
}

void GripperPanel::showStatus()
{
  const Status & s = latest_;
  QString state;
  if (!s.connected) {
    state = "<b style='color:#c0392b'>No communication</b>";
  } else if (s.fault_code != 0) {
    state = QString("<b style='color:#c0392b'>Fault 0x%1</b>")
      .arg(QString("%1").arg(s.fault_code, 2, 16, QChar('0')).toUpper());
  } else if (!s.activated) {
    state = s.activation_state == Status::STATE_ACTIVATING ?
      "<b>Activating...</b>" : "<b style='color:#d68910'>Not activated</b>";
  } else if (s.moving) {
    state = "<b>Moving</b>";
  } else if (s.object_detected) {
    state = "<b style='color:#1e8449'>Holding object</b>";
  } else {
    state = "<b>Ready</b>";
  }
  QString text = QString("%1 &nbsp; %2 &nbsp; width %3 mm &nbsp; current %4 A")
    .arg(QString::fromStdString(s.model), state)
    .arg(s.width * 1000.0, 0, 'f', 1)
    .arg(s.current, 0, 'f', 2);
  if (!s.fault_message.empty()) {
    text += "<br>" + QString::fromStdString(s.fault_message).toHtmlEscaped();
  }
  status_label_->setText(text);
}

void GripperPanel::showMessage(const QString & text)
{
  message_label_->setText(text);
}

void GripperPanel::sendGoal(double width_mm)
{
  if (!action_client_ || !have_status_) {
    showMessage("No status from the driver yet.");
    return;
  }
  if (!action_client_->action_server_is_ready()) {
    showMessage("Gripper action server is not available.");
    return;
  }
  GripperCommand::Goal goal;
  goal.command.position = {width_mm / 1000.0};
  goal.command.velocity = {speed_spin_->value() / 1000.0};
  goal.command.effort = {force_spin_->value()};

  // Callbacks may outlive the panel; QPointer becomes null once it is deleted.
  QPointer<GripperPanel> self(this);
  rclcpp_action::Client<GripperCommand>::SendGoalOptions options;
  options.goal_response_callback =
    [self](rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr handle) {
      if (!handle) {
        if (self) {
          QMetaObject::invokeMethod(
            self, "showMessage", Qt::QueuedConnection,
            Q_ARG(QString, "Goal rejected; see the driver log for the reason."));
        }
      }
    };
  options.result_callback =
    [self](const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult & wrapped) {
      QString text;
      switch (wrapped.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          text = wrapped.result->stalled ? "Stopped on an object." :
            "Reached the requested width.";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          text = "Goal canceled.";
          break;
        default:
          text = "Goal aborted or preempted; see the driver log.";
      }
      if (self) {
        QMetaObject::invokeMethod(self, "showMessage", Qt::QueuedConnection, Q_ARG(QString, text));
      }
    };
  showMessage(QString("Moving to %1 mm, %2 mm/s, %3 N...")
    .arg(width_mm, 0, 'f', 1).arg(speed_spin_->value(), 0, 'f', 0)
    .arg(force_spin_->value(), 0, 'f', 0));
  action_client_->async_send_goal(goal, options);
}

void GripperPanel::callTrigger(const QString & name)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
    name == "activate" ? activate_client_ : name == "stop" ? stop_client_ : reset_client_;
  if (!client || !client->service_is_ready()) {
    showMessage(QString("Service %1 is not available.").arg(name));
    return;
  }
  if (name == "activate") {
    showMessage("Activating: the fingers sweep their full stroke...");
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  QPointer<GripperPanel> self(this);
  client->async_send_request(
    request, [self, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString("%1: %2").arg(
        name, QString::fromStdString(response->message));
      if (self) {
        QMetaObject::invokeMethod(self, "showMessage", Qt::QueuedConnection, Q_ARG(QString, text));
      }
    });
}

}  // namespace robotiq_gripper_rviz

PLUGINLIB_EXPORT_CLASS(robotiq_gripper_rviz::GripperPanel, rviz_common::Panel)
