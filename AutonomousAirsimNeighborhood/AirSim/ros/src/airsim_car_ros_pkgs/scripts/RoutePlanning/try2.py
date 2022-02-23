#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def print_num(message):
    print("sub")

def talker():
    rospy.Subscriber("/current_pose", PoseStamped, print_num)
    goalPub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10) 
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    i = 0
    
    while not rospy.is_shutdown():   
        # if i < 1:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 1
        msg.pose.position.y = 1.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        goalPub.publish(msg)
        rate.sleep()
        i += 1
        print("pub")
    # rospy.spin()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass