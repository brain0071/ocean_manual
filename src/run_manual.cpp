#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/Attitude.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <Eigen/Dense>
#include <Eigen/QR> 
#include <cmath>


// ros::Publisher control_pub;

bool convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
{
	if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
		return false;
	}

	if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature)) {
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


void attMavlinkCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg, ros::Publisher* att_pub)
{
    mavlink_message_t mmsg;
    convert(*rmsg, mmsg);

    if (mmsg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(&mmsg, &packet);
 
        float roll, pitch, yaw;
        roll = packet.roll;
        pitch = packet.pitch; 
        yaw = packet.yaw;

        float w, x, y, z;
        w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
        x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    
        // send attitude 
        mavros_msgs::Attitude att_msg;
        att_msg.header.stamp = ros::Time::now();
        att_msg.orientation.x = x;
        att_msg.orientation.y = y;
        att_msg.orientation.z = z;
        att_msg.orientation.w = w;

        att_msg.body_att.x = roll;
        att_msg.body_att.y = pitch;
        att_msg.body_att.z = yaw;

        att_msg.body_rate.x = packet.rollspeed;
        att_msg.body_rate.y = packet.pitchspeed;
        att_msg.body_rate.z = packet.yawspeed;
   
        att_pub->publish(att_msg);
    }

    if (mmsg.msgid == MAVLINK_MSG_ID_ATTITUDE_TARGET) {
        
        
        mavlink_attitude_target_t packet;
        mavlink_msg_attitude_target_decode(&mmsg, &packet);
            
        // send attitude 
        mavros_msgs::AttitudeTarget att_target_msg;
        att_target_msg.header.stamp = ros::Time::now();
        att_target_msg.orientation.x = packet.q[0];
        att_target_msg.orientation.y = packet.q[1];
        att_target_msg.orientation.z = packet.q[2];
        att_target_msg.orientation.w = packet.q[3];

        att_target_msg.body_rate.x = packet.body_roll_rate;
        att_target_msg.body_rate.y = packet.body_pitch_rate;
        att_target_msg.body_rate.z = packet.body_yaw_rate;
   
        att_pub->publish(att_target_msg);
    }    
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "send_attitude");
	ros::NodeHandle nh;
    ros::Publisher att_pub = nh.advertise<mavros_msgs::Attitude>("/mavros/attitude", 10);
    ros::Subscriber sub = nh.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 10, boost::bind(attMavlinkCallback, _1, &att_pub));
	ros::spin();
	return 0;
}