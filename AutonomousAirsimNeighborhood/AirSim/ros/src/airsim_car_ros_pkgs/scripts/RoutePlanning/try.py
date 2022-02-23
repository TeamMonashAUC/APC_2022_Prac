import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    print("Yes!")

def main():
   rospy.init_node('try_broadcaster')
   rospy.Subscriber('move_base_simple/goal',
                  PoseStamped,
                  callback)
   rospy.spin()
   
if __name__ == '__main__':
   main()