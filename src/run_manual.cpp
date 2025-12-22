#include <stdio.h>
#include <iostream>

#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/mavlink.hpp>
#include <mavlink/v2.0/common/mavlink.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

class ManualControlNode : public rclcpp::Node
{

public:
  ManualControlNode() : Node("manual_control_node")
  {
    mavlink_sub = this->create_subscription<mavros_msgs::msg::Mavlink>(
        "/uas1/mavlink_source", rclcpp::QoS(10).best_effort(), [this](const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
        { this->mavlinkCallback(rmsg); });

    joystick_pub = this->create_publisher<mavros_msgs::msg::ManualControl>("/joystick", 10);

    // 20Hz timer -> 50ms interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ManualControlNode::main_callback, this));
    light_pub_ = this->create_publisher<std_msgs::msg::Bool>("/light", 10);
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gripper", 10);
    depth_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/depth", 10);
    pose_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_sub;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr joystick_pub;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr depth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // for mavlink Time synchronization
  bool have_time_sync_{false};
  uint32_t base_boot_ms_{0};
  rclcpp::Time base_ros_time_{0, 0, RCL_ROS_TIME};

  static bool light;
  static float gripper;
  static float roll;
  static float pitch;
  static float yaw;
  static float z;
  static float z_past;
  static float acce_x;
  static float acce_y;
  static tf2::Quaternion target_q;

  void mavlinkCallback(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
  {
    mavlink_message_t mmsg;
    convert(*rmsg, mmsg);
    if (mmsg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
      // RCLCPP_INFO(this->get_logger(), "Received MAVLINK_MSG_ID_MANUAL_CONTROL (msgid = %d)", mmsg.msgid);
      mavlink_manual_control_t packet;
      mavlink_msg_manual_control_decode(&mmsg, &packet);
      mavros_msgs::msg::ManualControl manual_msg;
      manual_msg.header.stamp = this->now();
      manual_msg.x = packet.x / 1000.0;
      manual_msg.y = packet.y / 1000.0;
      manual_msg.z = packet.z / 1000.0;
      manual_msg.r = packet.r / 1000.0;
      manual_msg.buttons = packet.buttons;
      manual_msg.buttons2 = packet.buttons2;
      manual_msg.enabled_extensions = packet.enabled_extensions;
      manual_msg.s = packet.s / 1000.0;
      manual_msg.t = packet.t / 1000.0;
      manual_msg.aux1 = packet.aux1 / 1000.0;
      manual_msg.aux2 = packet.aux2 / 1000.0;
      manual_msg.aux3 = packet.aux3 / 1000.0;
      manual_msg.aux4 = packet.aux4 / 1000.0;
      manual_msg.aux5 = packet.aux5 / 1000.0;
      manual_msg.aux6 = packet.aux6 / 1000.0;
      joystick_pub->publish(manual_msg);

      acce_x = ((packet.z / 1000.0) - 0.5) / 0.5 * 0.6;
      acce_y = packet.y / 1000.0 * 0.6;
      // RCLCPP_INFO(this->get_logger(), "acce_x: %.2f, acce_y: %.2f", acce_x, acce_y);

      switch (packet.buttons)
      {

      // D-pad
      // up(6144): Roll+
      // down(8192): Roll-
      // left(16384): Pitch+
      // right(32768): Pitch-
      case 6144:
      {
        // RCLCPP_INFO(this->get_logger(), "Roll+");
        roll += 0.5;
        break;
      }

      case 8192:
      {
        // RCLCPP_INFO(this->get_logger(), "Roll-");
        roll -= 0.5;
        break;
      }

      case 16384:
      {
        // RCLCPP_INFO(this->get_logger(), "Pitch+");
        pitch += 0.5;
        break;
      }

      case 32768:
      {
        // RCLCPP_INFO(this->get_logger(), "Pitch-");
        pitch -= 0.5;
        break;
      }

      // Button
      // X(4):Yaw+
      // B(2):Yaw-
      // Y(8):Z+
      // A(1):Z-
      case 4:
      {
        // RCLCPP_INFO(this->get_logger(), "Yaw+");
        yaw += 0.5;
        break;
      }

      case 2:
      {
        // RCLCPP_INFO(this->get_logger(), "Yaw-");
        yaw -= 0.5;
        break;
      }

      case 8:
      {
        // RCLCPP_INFO(this->get_logger(), "Z+");
        z += 0.001;
        break;
      }

      case 1:
      {
        // RCLCPP_INFO(this->get_logger(), "Z-");
        z -= 0.001;
        break;
      }

      // gripper
      // left(512): open
      // right(1024): close
      case 512:
      {
        // RCLCPP_INFO(this->get_logger(), "Gripper open");
        gripper -= (1900 - 1100) * 0.05;
        if (gripper < 1100)
        {
          gripper = 1100;
        }

        break;
      }

      case 1024:
      {
        // RCLCPP_INFO(this->get_logger(), "Gripper close");
        gripper += (1900 - 1100) * 0.05;
        if (gripper > 1900)
        {
          gripper = 1900;
        }

        break;
      }

      // light
      // left(16): open
      // right(64): close
      case 16:
      {
        // RCLCPP_INFO(this->get_logger(), "Light open");
        light = true;
        break;
      }

      case 64:
      {
        // RCLCPP_INFO(this->get_logger(), "Light close");
        light = false;
        break;
      }

      default:
        break;
      }
    }

    if (mmsg.msgid == MAVLINK_MSG_ID_SCALED_PRESSURE3)
    {
      // RCLCPP_INFO(this->get_logger(), "Received MAVLINK_MSG_ID_SCALED_PRESSURE3 (msgid = %d)", mmsg.msgid);
      mavlink_scaled_pressure3_t packet;
      mavlink_msg_scaled_pressure3_decode(&mmsg, &packet);
      // std_msgs::msg::Float32 depth_msg;
      // depth_msg.data = packet.press_abs;

      const float depth = packet.press_abs;
      const uint32_t boot_ms = packet.time_boot_ms;
      if (!have_time_sync_)
      {
        base_boot_ms_ = boot_ms;
        base_ros_time_ = this->get_clock()->now();
        have_time_sync_ = true;
      }
      const uint32_t dt_ms = boot_ms - base_boot_ms_;
      const rclcpp::Time stamp = base_ros_time_ + rclcpp::Duration(0, static_cast<int64_t>(dt_ms) * 1000000LL);
      geometry_msgs::msg::PointStamped depth_msg;
      out.header.stamp = stamp;
      out.header.frame_id = "base_link";
      out.point.x = 0.0;
      out.point.y = 0.0;
      out.point.z = depth;
      depth_pub_->publish(depth_msg);
    }
  }

  bool convert(const mavros_msgs::msg::Mavlink &rmsg, mavlink_message_t &mmsg)
  {
    if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0]))
    {
      return false;
    }

    if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature))
    {
      return false;
    }

    mmsg.magic = rmsg.magic;
    mmsg.len = rmsg.len;
    mmsg.incompat_flags = rmsg.incompat_flags;
    mmsg.compat_flags = rmsg.compat_flags;
    mmsg.seq = rmsg.seq;
    mmsg.sysid = rmsg.sysid;
    mmsg.compid = rmsg.compid;
    mmsg.msgid = rmsg.msgid;
    mmsg.checksum = rmsg.checksum;
    std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
    std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);
    return true;
  }

  void to_quaternion()
  {
    // to rad
    double R = roll * M_PI / 180.0f;
    double P = pitch * M_PI / 180.0f;
    double Y = yaw * M_PI / 180.0f;

    tf2::Quaternion q_new, q_past;
    q_new.setRPY(R, P, Y);
    q_past = target_q;

    if (q_past.dot(q_new) < 0)
    {
      q_new = tf2::Quaternion(-q_new.x(), -q_new.y(), -q_new.z(), -q_new.w());
    }
    double alpha = 0.05;
    target_q = q_past.slerp(q_new, alpha);
  }

  void main_callback()
  {
    // light
    std_msgs::msg::Bool light_msg;
    light_msg.data = light;
    light_pub_->publish(light_msg);

    // gripper
    std_msgs::msg::Float32 gripper_msg;
    gripper_msg.data = gripper;
    gripper_pub_->publish(gripper_msg);

    // RCLCPP_INFO(this->get_logger(), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Z: %.2f", roll, pitch, yaw, z);
    to_quaternion();
    // RCLCPP_INFO(this->get_logger(), "qw: %.2f, qx: %.2f, qy: %.2f, qz: %.2f", target_q.w(), target_q.x(), target_q.y(), target_q.z());

    float z_target;
    double alpha = 0.05;
    // smooth z
    z_target = (1 - alpha) * z_past + alpha * z;
    z_past = z_target;
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = acce_x;
    pose_msg.pose.position.y = acce_y;
    pose_msg.pose.position.z = z_target;
    pose_msg.pose.orientation.x = target_q.x();
    pose_msg.pose.orientation.y = target_q.y();
    pose_msg.pose.orientation.z = target_q.z();
    pose_msg.pose.orientation.w = target_q.w();
    pose_target_pub_->publish(pose_msg);
    return;
  }
};

bool ManualControlNode::light = false;
float ManualControlNode::gripper = 1100.0f;
float ManualControlNode::roll = 0.0f;
float ManualControlNode::pitch = 0.0f;
float ManualControlNode::yaw = 0.0f;
float ManualControlNode::acce_x = 0.0f;
float ManualControlNode::acce_y = 0.0f;
float ManualControlNode::z = 2.0f;
float ManualControlNode::z_past = 2.0f;
tf2::Quaternion ManualControlNode::target_q(0, 0, 0, 1);

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}