#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
from potential_field_planner import PotentialFieldPlanner
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json
import re

def remove_special_characters(input_string):
    # Define the regular expression pattern to match non-alphanumeric characters
    pattern = r'[^a-zA-Z0-9]'

    # Use the sub() function from the re module to replace non-alphanumeric characters with an empty string
    result = re.sub(pattern, '', input_string)

    return result

def compute_angle(vector1, vector2):
    dot_product = np.dot(vector1, vector2)  # Compute the dot product of the two vectors
    magnitude_product = np.linalg.norm(vector1) * np.linalg.norm(vector2)  # Compute the product of their magnitudes
    cosine_angle = dot_product / magnitude_product  # Compute the cosine of the angle
    
    # Use arccos to compute the angle in radians
    try:
        angle_radians = np.arccos(cosine_angle)
    except:
        angle_radians = 0.

    if vector1[1] < vector2[1]:
        angle_radians *= -1
    return angle_radians

class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate_number = 10
        self.rate = rospy.Rate(self.rate_number)  # Publisher frequency
        self.prev_lin_vel = np.array([0.0001] * 3) # Start with really small value

        # set the planner attributes
        self.time_step = 1 / self.rate_number
        self.k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.5]])
        self.k_rep     = 1
        self.vel_max   = 0.5
        self.planner_dic = {}
        self.goal = np.array([0., 0., 0.])
        self.robot_pos = np.array([0., 0., 0.])
        self.dist_min = 1. # minimum distance to obstacle

        # Initialize planner
        self.planner = PotentialFieldPlanner(self.goal, self.time_step, self.k_att, self.k_rep, self.dist_min)


    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        
        # Need to remove special characters from map keys to avoid potential key errors
        for k in self.map.keys():
            new_key = remove_special_characters(k)
            if 'obs' not in new_key:
                self.planner_dic[new_key] = np.array(self.map[k])
            else:
                self.planner_dic['obstacle'] = np.array(self.map[k])

        # Update goal
        if self.planner_dic['goal']:
            self.goal = np.concatenate([self.planner_dic['goal'], [0.]]) # Need 'three dimensional' goal position.
            rospy.loginfo(f"DEBUG: self.goal: {self.goal}")
            self.goal_dist = np.linalg.norm(self.goal[:2])
            rospy.loginfo(f"DEBUG: modified self.goal: {self.goal}")
            self.planner.set_target_pos(self.goal)
        else:
            rospy.logwarn("Goal not received or parsed in dictionary.")

        # Update obstacle
        if self.planner_dic['obstacle'] :
            self.obstacle_pos = np.concatenate([self.planner_dic['obstacle'], [0.]]) # Need 'three dimensional' obstacle position.
            self.planner.set_obstacle_distance(self.dist_min)
            self.planner.set_obstacle_position(self.obstacle_pos)
            rospy.loginfo(f"DEBUG: obstacle in planner_dic")
        else:
            # self.obstacle_pos = None
            # self.planner.set_obstacle_position(self.obstacle_pos)
            rospy.loginfo(f"DEBUG: obstacle NOT in planner_dic")

        # Twist
        self.cmd = geometry_msgs.msg.Twist()

        # Update the current command
        # Stop at 0.1 meter from goal
        if self.goal_dist < 0.1:
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = 0.
        else:
            # Potential Field Planner commands
            pos_des, lin_vel =	self.planner.get_avoidance_force(self.robot_pos)
            self.cmd.linear.x = lin_vel[0] * 0.5
            self.cmd.linear.y = lin_vel[1] * 0.5
            # angle = compute_angle(lin_vel[:2], self.prev_lin_vel[:2]) 
            # self.cmd.angular.z = np.clip(angle, -.2, .2)
            # self.prev_lin_vel = lin_vel
            angle_error = np.arctan2(self.goal[1], self.goal[0]) 
            self.cmd.angular.z = np.clip(angle_error, -.2, .2)

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
