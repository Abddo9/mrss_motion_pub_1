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

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # Need to remove special characters from map keys to avoid potential key errors
        for k in self.map.keys():
            new_key = remove_special_characters(k)
            if 'obs' not in new_key:
                self.planner_dic[new_key] = np.array(self.map[k])
            else:
                self.planner_dic['obstacle'] = np.array(self.map[k])

        goal = self.planner_dic['goal']

        # Twist
        self.cmd = geometry_msgs.msg.Twist()

        # TODO BEGIN MRSS: Update the current command
        n_goal = np.linalg.norm(goal)
        if n_goal < 0.1:
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = 0.
        else:
            # Linear planner commands
            self.cmd.linear.x = goal[0]/n_goal * 0.15
            self.cmd.linear.y = goal[1]/n_goal * 0.15
            angle_error = np.arctan2(goal[1], goal[0]) 
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
