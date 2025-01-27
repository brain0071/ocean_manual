#!/usr/bin/env python

import rospy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from mavros_msgs.msg import AttitudeTarget

class Att_TargetWrapper:
    
    def __init__(self):
        
        self.att_target_pub = rospy.Publisher(
            "/mavros/attitude_target", AttitudeTarget, queue_size=10
        )
        
        rate = rospy.Rate(10)
        while True:
            self.target_callback()
            rate.sleep()
            
    def target_callback(self):
        
        roll_target = 0
        pitch_target = 0
        yaw_target = 0
        
        roll_rad = math.radians(roll_target)
        pitch_rad = math.radians(pitch_target)
        yaw_rad = math.radians(yaw_target)
        
        r = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad], degrees=False)
        q = r.as_quat()  # return [x, y, z, w]
        
        att_target = AttitudeTarget()
        att_target.orientation.w = q[3]
        att_target.orientation.x = q[0]
        att_target.orientation.y = q[1]
        att_target.orientation.z = q[2]
        
        att_target.body_rate.x = 0
        att_target.body_rate.y = 0
        att_target.body_rate.z = 0
        
        self.att_target_pub.publish(att_target)
        
        
def main():
    rospy.init_node("send attitude target")
    Att_TargetWrapper()

if __name__ == "__main__":
    main()
