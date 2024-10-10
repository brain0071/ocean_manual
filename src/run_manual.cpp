#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/WheelOdomStamped.h>
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


void mavlinkCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg, ros::Publisher* manual_pub, ros::Publisher* motor_pub, ros::Publisher* joystick_pub)
{
    mavlink_message_t mmsg;
    convert(*rmsg, mmsg);
    if (mmsg.msgid != MAVLINK_MSG_ID_MANUAL_CONTROL) {
        ROS_ERROR("A Error message not a MAVLINK_MSG_ID_MANUAL_CONTROL.");
        return;
    }
    mavlink_manual_control_t packet;
    mavlink_msg_manual_control_decode(&mmsg, &packet);
    float x, y, z, roll, pitch, yaw;
    // collect attitude data x:roll y:pitch z:yaw r: 0
    // buttons 4096/8192: x+/x-
    // buttons 16384/32768: z+/z-
    // use 60% power
    // deadzone:0.05
    printf("packet.x: %f\n", packet.x);

    if (std::abs(packet.x) < 0.05) {
        roll = 0; 
    } else {
        roll = packet.x / 1000.0 * 0.6;  
    }

    printf("roll: %f\n", roll);

    if (std::abs(packet.y) < 0.05) {
        pitch = 0; 
    } else {
        pitch = -packet.y / 1000.0 * 0.6; 
    }

    if (std::abs(packet.r) < 0.05) {
        yaw = 0; 
    } else {
        yaw = -packet.r / 1000.0 * 0.6;
    }
    
    switch (packet.buttons) {
    
    case 2048: { 
        x = 1 * 0.6;
        y = 0;
        z = 0; 
        break; 
        }
    
    case 4096: { 
        x = -1 * 0.6; 
        y = 0;
        z = 0; 
        break; 
        }

    case 8: { 
        x = 0;
        y = 0;
        z = 1 * 0.6;
        break; 
        }
    
    case 1: { 
        x = 0;
        y = 0;
        z = -1 * 0.6; 
        break; 
        }
    
    default: 
        x = 0;
        y = 0;
        z = 0;
        break;
    }
    
    // send joystick signals
    mavros_msgs::ManualControl joystick_msg;
    joystick_msg.header.stamp = ros::Time::now();
    joystick_msg.x = packet.x;
    joystick_msg.y = packet.y;
    joystick_msg.z = packet.z;
    joystick_msg.r = packet.r;
    joystick_msg.buttons = packet.buttons;
    joystick_pub->publish(joystick_msg);

    // send control 
    mavros_msgs::WheelOdomStamped msg;
    msg.data.clear();
    msg.header.stamp = ros::Time::now();
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    manual_pub->publish(msg);

    // control vector
    Eigen::Matrix<double, 6, 1> control_vector;
    control_vector << x, y, z, roll, pitch, yaw;
    // TODO:

    // allocation matrix [6,8]
    Eigen::MatrixXd allo_matrix(6, 8);
    allo_matrix << 0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736,  // x
                   -0.57736, -0.57736, 0.57736, 0.57736, 0.57736, 0.57736, -0.57736, -0.57736,  // y
                   0.57736, -0.57736, -0.57736, 0.57736, -0.57736, 0.57736, 0.57736, -0.57736,  // z
                   0.199105, 0.199105, 0.199105, 0.199105, -0.199105, -0.199105, -0.199105, -0.199105, // r
                   -0.217233, 0.217233, 0.217233, -0.217233, -0.217233, 0.217233, 0.217233, -0.217233, // p
                   -0.210211, -0.210211, 0.210211, 0.210211, -0.210211, -0.210211, 0.210211, 0.210211; // y
    
    Eigen::MatrixXd allo_matrix_pinv  = allo_matrix.completeOrthogonalDecomposition().pseudoInverse(); 
    
    // mpc code
    // motor_percent = f_max(8*8)^-1 * A^-1 * tau_max(6*6) * u 
    // motor_ = np.dot(np.linalg.inv(max_motor_),
    //          np.dot(np.linalg.pinv(self.motor_matrix), 
    //          np.dot(np.diag(self.max_forceMoment), next_control)))
    
    // max force of each motor 
    Eigen::DiagonalMatrix<double, 8> max_motor_matrix;
    double motor_max = 3.5 * 9.8;
    max_motor_matrix.diagonal() << motor_max, motor_max, motor_max, motor_max, motor_max, motor_max, motor_max, motor_max;
    Eigen::DiagonalMatrix<double, 8> motor_inverse_matrix = max_motor_matrix.inverse();

    // // max force and moment of each freedom degree
    Eigen::DiagonalMatrix<double, 6> max_forceMoment_matrix;
    max_forceMoment_matrix.diagonal() << 158.4276, 158.4276, 158.4276, 54.6344, 59.6087, 57.6819;

    Eigen::Matrix<double, 8, 1> motor_vector;
    motor_vector =  motor_inverse_matrix * allo_matrix_pinv * max_forceMoment_matrix * control_vector;
    
    // // send control 
    mavros_msgs::ActuatorControl motor_msg;
    msg.header.stamp = ros::Time::now();
    motor_msg.controls[0] = motor_vector[0];
    motor_msg.controls[1] = motor_vector[1];
    motor_msg.controls[2] = motor_vector[2];
    motor_msg.controls[3] = motor_vector[3];
    motor_msg.controls[4] = motor_vector[4];
    motor_msg.controls[5] = motor_vector[5];
    motor_msg.controls[6] = motor_vector[6];
    motor_msg.controls[7] = motor_vector[7];
    motor_pub->publish(motor_msg);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "manual");
	ros::NodeHandle nh;

    ros::Publisher manual_pub = nh.advertise<mavros_msgs::WheelOdomStamped>("/manual_control", 10);
    ros::Publisher motor_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
    // 20240924: get joystick
    ros::Publisher joystick_pub = nh.advertise<mavros_msgs::ManualControl>("/joystick", 10);
    ros::Subscriber sub = nh.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 10, boost::bind(mavlinkCallback, _1, &manual_pub, &motor_pub, &joystick_pub));
	ros::spin();
	return 0;
}