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

        # TODO BEGIN MRSS: Add attributes (If needed)
        # set the planner attributes
        self.time_step = 0.001
        self.k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.5]])
        self.k_rep     = 1
        self.vel_max   = 0.5
        self.planner = None
        self.obstacle = None
        self.goal = None
        # END MRSS
        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        self.planner_dic = {}

        # TODO BEGIN MRSS: Use map for planning
        for k in self.map.keys():
            new_key = remove_special_characters(k)
            if 'obs' not in new_key:
                self.planner_dic[new_key] = np.array(self.map[k])
            else:
                self.planner_dic['obstacle'] = np.array(self.map[k])
        try:
            self.goal = np.concatenate([self.planner_dic['goal'], [0.]])
            self.planner   = PotentialFieldPlanner(self.goal, 1 / self.rate_number, self.k_att, self.k_rep, self.vel_max)
            n_goal = np.linalg.norm(self.goal)
            if 'obstacle' in self.planner_dic.keys():
                self.obstacle = np.concatenate([self.planner_dic['obstacle'], [0.]])
            else:
                self.obstacle = None
            rospy.logwarn("***************")


            # Potential Field Planner
            

                
            # END MRSS

            # Twist
            self.cmd = geometry_msgs.msg.Twist()

            # TODO BEGIN MRSS: Update the current command
        
            if n_goal < 0.1 or self.goal is None:
                self.cmd.linear.x = 0.
                self.cmd.linear.y = 0.
                self.cmd.angular.z = 0.
                rospy.logwarn("***eeee****")
            else:
                # Vanilla commands
                # self.cmd.linear.x = goal[0]/n_goal * 0.15
                # self.cmd.linear.y = goal[1]/n_goal * 0.15
                # angle_error = np.arctan2(goal[1], goal[0]) 
                # self.cmd.angular.z = np.clip(angle_error, -.2, .2)
                # rospy.logwarn("***eeeaasdfe****")
                # Potential Field Planner commands
                if self.obstacle is not None:
                    self.planner.set_obstacle_distance(1.5)
                    self.planner.set_obstacle_position(self.obstacle)
                    pos_des, lin_vel =	self.planner.get_avoidance_force([0., 0., 0.])
                else:
                    pos_des, lin_vel =	self.planner.get_attractive_force([0., 0., 0.])
                self.cmd.linear.x = lin_vel[0]
                self.cmd.linear.y = lin_vel[1]
                angle = np.arctan2(pos_des[1], pos_des[0]) 
                self.cmd.angular.z = np.clip(angle, -.2, .2)
                
            # END MRSS
        except:
            pass

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
