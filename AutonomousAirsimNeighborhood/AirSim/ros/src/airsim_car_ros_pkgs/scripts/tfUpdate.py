#!/usr/bin/env python  

from math import pi
import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


def publish_pose(pos,quat):

   pStamp = PoseStamped()
   pStamp.header.frame_id = "/map"
   pStamp.header.stamp = rospy.Time.now()
   pStamp.pose.position.x = pos.x
   pStamp.pose.position.y = pos.y
   pStamp.pose.position.z = pos.z
   pStamp.pose.orientation.x = quat[0]
   pStamp.pose.orientation.y = quat[1]
   pStamp.pose.orientation.z = quat[2]
   pStamp.pose.orientation.w = quat[3]

   posePub.publish(pStamp)

def publish_vel(twist):
   line = twist.linear
   angle = twist.angular

   tStamp = TwistStamped()
   tStamp.header.frame_id = "PhysXCar/odom_local_enu"
   tStamp.header.stamp = rospy.Time.now()
   tStamp.twist.linear.x = -line.x
   tStamp.twist.linear.y = line.y
   tStamp.twist.linear.z = line.z
   tStamp.twist.angular.x = angle.x
   tStamp.twist.angular.y = angle.y
   tStamp.twist.angular.z = angle.z

   velPub.publish(tStamp)

def update_pose(msg):
   pose = msg.pose.pose
   pos = pose.position
   orient = pose.orientation
   twist = msg.twist.twist

   # Adjust orientation
   euler = euler_from_quaternion((orient.x,orient.y,-orient.z,orient.w))
   euler = np.asarray(euler)
   euler[2] = euler[2] + pi/2
   quat = quaternion_from_euler(euler[0],euler[1],euler[2])

   # Send pose and orientation to tf
   br = tf.TransformBroadcaster()
   br.sendTransform((pos.x,pos.y,pos.z),
               (quat),
               rospy.Time.now(),
               "PhysXCar/odom_local_enu",
               "map")

   # Publish PoseStamped and TwistStamped to topics
   publish_pose(pos,quat)
   publish_vel(twist)


def main():
   rospy.init_node('PhysXcar_broadcaster')
   rospy.Subscriber("/airsim_node/PhysXCar/odom_local_enu",
                  Odometry,
                  update_pose)
   rospy.spin()
   
if __name__ == '__main__':
   
   posePub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
   velPub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
   main()