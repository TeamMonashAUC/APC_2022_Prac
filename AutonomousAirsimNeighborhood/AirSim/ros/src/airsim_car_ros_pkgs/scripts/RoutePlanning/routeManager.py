#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from GeneticAlgo import geneticAlgorithm
from carControl import Car
from math import sqrt
import numpy as np
import pickle

class RouteManager:

    def __init__(self, coord, img, map = None):

        # Start Genetic Algorithm to find best route
        best_route = geneticAlgorithm(coordinates = coord, popSize = 100, eliteSize = 10, mutationRate = 0.01, generations = 500, visualise = True, image = img)[1:]
        # Current carControl cannot reverse or U-turn. So we force the route plan to start in the forward direction of car
        if best_route[0] != (512,313):
            best_route = best_route[::-1]

        if map is not None:
            self.route = [map[i] for i in best_route]
        else:   
            self.route = best_route   
        
        # Delete some coordinates from self.route
        index = [2,3,5,6,12,14,15]
        self.route = [point for i, point in enumerate(self.route) if i not in index]

        self.current_goal = 0
        self.distance = 0
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1) 
        print("Total goals: {}".format(len(self.route)))

    def publish_goal(self):
        
        # Publish next goal when current pose is within 3m of current goal or if no goal has been published yet
        
        goal = self.route[self.current_goal]
        
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
            
    def distance_to_goal(self, pose):
        
        # Calculate euclidean distance from current pose to goal
        pos = pose.pose.position
        goal = self.route[self.current_goal]

        self.distance = sqrt((goal[0] - pos.x)**2 + (goal[1] - pos.y)**2)
        
        # Increment current_goal when current pose is within 7m of current goal 
        if self.distance < 7: 
            print("Reached goal {}".format(self.current_goal + 1))
            
            if self.current_goal < len(self.route):
                self.current_goal += 1
            else:
                print("Reached all goals!")
                
                


def pub_and_sub():
    
    rospy.init_node('routeManager', anonymous=True)
    rospy.loginfo("Starting routeManager script")
    rospy.Subscriber("/current_pose", PoseStamped, manager.distance_to_goal)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown(): 
        manager.publish_goal()
        rate.sleep()


if __name__ == '__main__':

    # Load coordinates of image
    with open("Coordinates.pkl","rb") as f:
        coord =pickle.load(f)
    f.close()
    coord_mod = np.vstack((coord['green'],coord['red']))

    # Mapping between image coordinates and actual map coordinates
    mapping = { 
        (512,313):(-49.7,3.3), 
        (496,259):(-80.1,-6.8), 
        (512,248):(-85.7,3), 
        (499,238):(-93,-4), 
        (433,250):(-87.5,-39.2), 
        (493,35):(-208,8,-8.9), 
        (430,22):(-215.6,-42.6), 
        (426,45):(-203.1,-45), 
        (330,23):(-214.9,-96.6), 
        (292,22):(-214,-116.8), 
        (33,52):(-200.1,-253.7), 
        (49,247):(-86.4,-247.2), 
        (62,343):(-35.5,-239.7), 
        (235,250):(-84.5,-148.3), 
        (268,236):(-96.8,-131.4), 
        (286,250):(-87,-119.4), 
        (268,462):(35,-131.4), 
        (487,485):(48,-10.6), 
        (500,457):(33.8,-4.5)
    }
    
    manager = RouteManager(coord_mod, "MapImg.png", mapping)
  
    pub_and_sub()

    PhysXCar = Car()



