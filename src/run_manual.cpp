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
    subscription_ = this->create_subscription<mavros_msgs::msg::Mavlink>(
      "/uas1/mavlink_sink", rclcpp::QoS(10).best_effort(), [this](const mavros_msgs::msg::Mavlink::SharedPtr rmsg) {
    this->mavlinkCallback(rmsg);});
  }

  private:
  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr subscription_;


  void mavlinkCallback(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
  {
    RCLCPP_INFO(this->get_logger(), "Update");
    mavlink_message_t mmsg;
    convert(*rmsg, mmsg);
    if (mmsg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) 
    {
      RCLCPP_INFO(this->get_logger(), "Received MAVLINK_MSG_ID_MANUAL_CONTROL (msgid = %d)", mmsg.msgid);
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