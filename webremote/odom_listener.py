#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped  

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
    
        self.subscription_odom = self.create_subscription( #for orientation
            Odometry,
            '/pose', 
            self.callback_global, qos_profile_sensor_data
        ) 
        
        print('node started') 
        self.global_xy=[0,0] 
 

 
    def callback_global(self, msg): 
        """
        current pos
        convert current x,y to lat,lon and serve on the endpoint /gps
        """
        pose=msg.pose.pose.position  
        self.global_xy=[pose.x, pose.y]
        print('global: %0.2f %0.2f' % (self.global_xy[0], self.global_xy[1]) ) 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber() 
    try:
        rclpy.spin(minimal_subscriber) 
    except KeyboardInterrupt:
        print('keyboard interrupt detected.')  

    minimal_subscriber.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
