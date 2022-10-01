#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import numpy as np

import signal 
import threading
from flask import Flask
from flask import Flask, redirect
from flask import Flask, render_template, request

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from nav_msgs.msg import Path
import time

# for motor state 1 command
from p2os_msgs.msg import MotorState

'''
possible problem
many remote can connect to this node.
so, they are in different thread

publish_motor are called from different webconnection


'''


class PioneerPub(Node):
    
    def __init__(self):
        super().__init__('pioneer_remote_websocket')

        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.motor_state_pub= self.create_publisher(MotorState, '/cmd_motor_state', 10)
        self.motor_power=False
        
        self.t1=time.time()
        print('origin time=', self.t1)
        self.id=0

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self): 
        """
        if no signal from remote controller then send 0 or 1 to the motor.
        """

        self.i += 1
        ct=time.time()
        dt=ct - self.t1
        if dt>3:
            print('sending stop')
            ms=MotorState()
            ms.state=1
            self.motor_state_pub.publish(ms)
            self.publish_twist('stop')


    def publish_motor_state(self, state):
        ms=MotorState()
        ms.state=state
        self.motor_state_pub.publish(ms)
        self.t1=time.time()


    def publish_twist(self, d, speed=0.5):
        if speed<0:
            speed=0
        elif speed>10:
            speed=10



        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        if d=='up':
            twist.linear.x = speed
        elif d=='down':
            twist.linear.x = - speed
        elif d=='left':
            twist.angular.z =  speed
        elif d=='right':
            twist.angular.z = - speed

        elif d=='stop':
            #sends all 0
            pass
        else:
            return
        self.cmdvel_publisher.publish(twist)



    def callback_odom_global(self, msg): 
        global robot_z
        """
        for robot orientation. 
        """
        print('callback_odom_global')
        

def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler
    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    # if prev_sigint_handler is not None:
    #     prev_sigint_handler(signal)






app = Flask(__name__, template_folder='template')
import logging
log = logging.getLogger('werkzeug')
log.disabled = True

@app.route('/')  
def home():  
    return render_template('index.html') 

 
@app.route('/power', methods = ['POST'])  
def set_power(): 
    global ros2_node
    d=request.form['power']
    print('power: ', d)
    d=int(d)
    if d==1:
        ros2_node.motor_power=True
    else:
        ros2_node.motor_power=False

    print('motor_power=', ros2_node.motor_power)

    ros2_node.publish_motor_state(d) 
 
    return 'thanks' 

@app.route('/remote', methods = ['POST'])  
def set_remote(): 
    global ros2_node
    direction=request.form['direction']
    speed=request.form['speed']
    speed=float(speed)
    # print('direction: ', direction, ' speed=', speed)

    ros2_node.publish_twist(direction, speed)  
 
    return 'thanks' 

 



rclpy.init(args=None)  
ros2_node=None
ros2_node = PioneerPub()

prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)
def main():
    global app
    global ros2_node   
    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
   
    app.run(debug = True, host='0.0.0.0', use_reloader=False)

if __name__ == '__main__': 
    main()
    
"""
culprt found
https://stackoverflow.com/questions/25504149/why-does-running-the-flask-dev-server-run-itself-twice

The Werkzeug reloader spawns a child process so that it can restart that process each time 
your code changes. Werkzeug is the library that supplies Flask with the development 
server when you call app.run().


"""
