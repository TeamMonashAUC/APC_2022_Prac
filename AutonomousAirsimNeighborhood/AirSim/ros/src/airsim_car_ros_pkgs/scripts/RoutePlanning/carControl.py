#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from airsim_car_ros_pkgs.msg import CarCmd, ControlCommandStamped
import airsim
import math

class Car:

    def __init__(self, max_vel = 5):
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(True)
        self.carPub = rospy.Publisher('/airsim_node/PhysXCar/car_cmd_body_frame', CarCmd, queue_size=1)

        self.controls = CarCmd()
        self.current_vel = 0
        self.prev_vel = 0
        self.max_vel = max_vel
        self.brake = False

        self.listener()
    
    def controlCar(self, msg):
        
        if self.brake == True:
            self.controls.brake = 1
        
        self.controls.is_manual_gear = False
        self.controls.manual_gear = 0
        self.controls.gear_immediate = True
        
        # Set throttle of vehicle using PD controller
        K_p = 0.02   # Proportional gain
        K_d = 0.015  # Derivative gain
        
        error = (self.max_vel - self.current_vel)/10
        error_dt = -(self.current_vel - self.prev_vel)
        self.prev_vel = self.current_vel
        
        self.controls.throttle += K_p * error + K_d * error_dt

        # print(self.current_vel)

        # Set steering of vehicle using values from /ctrl_raw
        self.controls.steering = -msg.cmd.steering_angle

        self.carPub.publish(self.controls)

    def setVelocity(self,twist):

        # Set current velocity of car
        vel_x = twist.twist.linear.x
        vel_y = twist.twist.linear.y
        self.current_vel = math.sqrt(vel_x**2 + vel_y**2)

    def brake(self):

        # Stop the car
        self.max_vel = 0
        self.brake = True


    def listener(self):
    
        rospy.init_node('carControl', anonymous=True)
        rospy.Subscriber("/current_velocity", TwistStamped, self.setVelocity)
        rospy.Subscriber("/ctrl_raw", ControlCommandStamped, self.controlCar)
        rospy.loginfo("Starting car control script")
        rospy.spin()


if __name__ == '__main__':
    
    # client = airsim.CarClient()
    # client.confirmConnection()
    # client.enableApiControl(True)

    PhysXCar = Car()
    
    # carPub = rospy.Publisher('/airsim_node/PhysXCar/car_cmd_body_frame', CarCmd, queue_size=1)
    # listener()