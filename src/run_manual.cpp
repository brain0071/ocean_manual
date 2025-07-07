#include <stdio.h>
#include <iostream>

#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/wheel_odom_stamped.hpp>
#include <mavros_msgs/msg/mavlink.hpp>
#include <mavlink/v2.0/common/mavlink.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class ManualControlNode : public rclcpp::Node {
  
  public:
  ManualControlNode() : Node("manual_control_node")
  {
    mavlink_sub = this->create_subscription<mavros_msgs::msg::Mavlink>(
      "/uas1/mavlink_source", rclcpp::QoS(10).best_effort(), [this](const mavros_msgs::msg::Mavlink::SharedPtr rmsg) {
    this->mavlinkCallback(rmsg);});

    joystick_pub = this->create_publisher<mavros_msgs::msg::ManualControl>(
    "/joystick", 10);  

  }


  private:

  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_sub;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr joystick_pub;

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
      
      switch (packet.buttons) {
        
        // D-pad
        // up(6144): Roll+
        // down(8192): Roll- 
        // left(16384): Pitch+ 
        // right(32768): Pitch-
        case 6144: {
          RCLCPP_INFO(this->get_logger(), "Roll+");
          break; 
        }
    
        case 8192: { 
          RCLCPP_INFO(this->get_logger(), "Roll-");
          break; 
        }

        case 16384: { 
          RCLCPP_INFO(this->get_logger(), "Pitch+");
          break; 
        }
    
        case 32768: { 
          RCLCPP_INFO(this->get_logger(), "Pitch-");
          break; 
        }

        // Button
        // X(4):Yaw+ 
        // B(2):Yaw-  
        // Y(8):Z+ 
        // A(1):Z-
        case 4: {
          RCLCPP_INFO(this->get_logger(), "Yaw+");
          break; 
        }
    
        case 2: { 
          RCLCPP_INFO(this->get_logger(), "Yaw-");
          break; 
        }

        case 8: { 
          RCLCPP_INFO(this->get_logger(), "Z+");
          break; 
        }
    
        case 1: { 
          RCLCPP_INFO(this->get_logger(), "Z-");
          break; 
        }

        // gripper 
        // left(512): open 
        // right(1024): close
        case 512: {
          RCLCPP_INFO(this->get_logger(), "Gripper open");
          break; 
        }
    
        case 1024: { 
          RCLCPP_INFO(this->get_logger(), "Gripper close");
          break; 
        }
        
        default: 
          break;
        }

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

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}