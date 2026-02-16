#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "contactile_gripper_ros2/srv/gripper_command.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class GripperActionBridge : public rclcpp::Node
{
public:
  using GripperCommandAction = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
  using GripperSrv = contactile_gripper_ros2::srv::GripperCommand;

  GripperActionBridge()
  : Node("contactile_gripper_action_bridge")
  {
    service_name_ = this->declare_parameter<std::string>("service_name", "/contactile_gripper_comms");
    command_name_ = this->declare_parameter<std::string>("command_name", "PC_MOVE_TO_WIDTH");
    scale_ = this->declare_parameter<double>("scale", 1.0);

    // --- Gripper state stream ---
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/contactile_data_stream");
    joint_name_  = this->declare_parameter<std::string>("joint_name", "gr_b_to_l1");
    state_index_ = this->declare_parameter<int>("state_index", 1);
    state_scale_  = this->declare_parameter<double>("state_scale", 1.0);
    state_offset_ = this->declare_parameter<double>("state_offset", 0.0);

    // --- JointState merge I/O ---
    // Read robot joints from here (usually /joint_states from ur_ros2_control + joint_state_broadcaster)
    input_joint_states_topic_  = this->declare_parameter<std::string>("input_joint_states_topic", "/joint_states");
    // Publish merged joints here (UR + gripper in one message)
    output_joint_states_topic_ = this->declare_parameter<std::string>("output_joint_states_topic", "/joint_states_combined");

    // Publish rate for merged JointState (Hz). If 0 -> publish only on incoming msgs.
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);

    // QoS: robust for JointState streams (avoid transient_local mismatch headaches)
    auto js_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();

    joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(output_joint_states_topic_, js_qos);

    // Subscribe to robot joint states (UR)
    robot_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_joint_states_topic_, js_qos,
      std::bind(&GripperActionBridge::robot_js_cb, this, std::placeholders::_1)
    );

    // Subscribe to gripper state stream
    state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      state_topic_, 10,
      std::bind(&GripperActionBridge::state_cb, this, std::placeholders::_1)
    );

    // Optional periodic publisher (recommended)
    if (publish_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GripperActionBridge::publish_merged_if_ready, this)
      );
    }

    RCLCPP_INFO(get_logger(), "Robot JS sub: %s (sensor_msgs/JointState)", input_joint_states_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Gripper state sub: %s (Float64MultiArray), index=%d", state_topic_.c_str(), state_index_);
    RCLCPP_INFO(get_logger(), "Merged JS pub: %s (sensor_msgs/JointState), gripper joint=%s",
                output_joint_states_topic_.c_str(), joint_name_.c_str());
    RCLCPP_INFO(get_logger(), "Merged publish rate: %.2f Hz", publish_rate_hz_);

    // Create service client
    srv_client_ = this->create_client<GripperSrv>(service_name_);

    // Create action server on /gripper_cmd
    action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      this,
      "gripper_cmd",
      std::bind(&GripperActionBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperActionBridge::handle_cancel, this, std::placeholders::_1),
      std::bind(&GripperActionBridge::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Gripper Action Bridge ready.");
    RCLCPP_INFO(get_logger(), " action: /gripper_cmd (control_msgs/GripperCommand)");
    RCLCPP_INFO(get_logger(), " service: %s (contactile_gripper_ros2/srv/GripperCommand)", service_name_.c_str());
    RCLCPP_INFO(get_logger(), " command_name: %s, scale: %g", command_name_.c_str(), scale_);
  }

private:
  // ---- Params ----
  std::string service_name_;
  std::string command_name_;
  double scale_{1.0};

  std::string state_topic_;
  std::string joint_name_;
  int state_index_{1};
  double state_scale_{1.0};
  double state_offset_{0.0};

  std::string input_joint_states_topic_;
  std::string output_joint_states_topic_;
  double publish_rate_hz_{50.0};

  // ---- ROS entities ----
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_js_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Client<GripperSrv>::SharedPtr srv_client_;
  rclcpp_action::Server<GripperCommandAction>::SharedPtr action_server_;

  // ---- Cached state for merge ----
  std::mutex mtx_;
  sensor_msgs::msg::JointState last_robot_js_;
  bool have_robot_{false};

  double last_gripper_pos_{0.0};
  bool have_gripper_{false};

  // ---- Helpers ----
  static int find_joint_index(const std::vector<std::string>& names, const std::string& target)
  {
    auto it = std::find(names.begin(), names.end(), target);
    if (it == names.end()) return -1;
    return static_cast<int>(std::distance(names.begin(), it));
  }

  void robot_js_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_robot_js_ = *msg;
    have_robot_ = true;

    // If we don't run a timer, publish on incoming robot joint states
    if (!publish_timer_) {
      publish_merged_locked_();
    }
  }

  void state_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (state_index_ < 0 || state_index_ >= static_cast<int>(msg->data.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "state_index %d out of range (len=%zu)", state_index_, msg->data.size());
      return;
    }

    const double raw = msg->data[static_cast<size_t>(state_index_)];
    const double pos = raw * state_scale_ + state_offset_;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_gripper_pos_ = pos;
      have_gripper_ = true;

      // If we don't run a timer, publish on incoming gripper state too
      if (!publish_timer_) {
        publish_merged_locked_();
      }
    }
  }

  void publish_merged_if_ready()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    publish_merged_locked_();
  }

  void publish_merged_locked_()
  {
    if (!have_robot_ || !have_gripper_) {
      return;  // wait until both streams are available
    }

    sensor_msgs::msg::JointState out = last_robot_js_;
    out.header.stamp = this->now();

    // Ensure arrays are consistent
    const size_t n = out.name.size();
    if (out.position.size() != n) out.position.resize(n, 0.0);
    if (!out.velocity.empty() && out.velocity.size() != n) out.velocity.resize(n, 0.0);
    if (!out.effort.empty()   && out.effort.size()   != n) out.effort.resize(n, 0.0);

    const int idx = find_joint_index(out.name, joint_name_);
    if (idx >= 0) {
      out.position[static_cast<size_t>(idx)] = last_gripper_pos_;
      // if velocity/effort arrays exist, we keep whatever was there (or 0 if resized)
    } else {
      out.name.push_back(joint_name_);
      out.position.push_back(last_gripper_pos_);
      if (!out.velocity.empty()) out.velocity.push_back(0.0);
      if (!out.effort.empty())   out.effort.push_back(0.0);
    }

    joint_state_pub_->publish(out);
  }

  // ---- Action bridge (MoveIt -> service) ----
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const GripperCommandAction::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<GripperCommandAction::Result>();

    if (!srv_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Service not available: %s", service_name_.c_str());
      goal_handle->abort(result);
      return;
    }

    const auto goal = goal_handle->get_goal();

    auto req = std::make_shared<GripperSrv::Request>();
    req->command_name = command_name_;
    req->command_argument = static_cast<double>(goal->command.position) * scale_;

    RCLCPP_INFO(get_logger(), "Goal: position=%f -> command=%s arg=%f",
                goal->command.position, req->command_name.c_str(), req->command_argument);

    auto future = srv_client_->async_send_request(req);
    auto status = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future, std::chrono::seconds(3));

    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Service call failed/timed out");
      goal_handle->abort(result);
      return;
    }

    goal_handle->succeed(result);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperActionBridge>());
  rclcpp::shutdown();
  return 0;
}
