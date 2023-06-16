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

        self.goal = np.array([0., 0., 0.])
        self.planner_dic = {}

    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        rospy.loginfo(f"DEBUG: map callback.\nMessage: {msg}\nExtracted map:{self.map}")
        # Need to remove special characters from map keys to avoid potential key errors
        for k in self.map.keys():
            new_key = remove_special_characters(k)
            if 'obs' not in new_key:
                rospy.loginfo(f"DEBUG: new_key has no obs: {new_key}")
                self.planner_dic[new_key] = np.array(self.map[k])
            else:
                rospy.loginfo(f"DEBUG: new_key HAS obs and must be obstacle: {new_key}")
                self.planner_dic['obstacle'] = np.array(self.map[k])

        rospy.loginfo(f"DEBUG: self.goal: {self.goal}")
        self.goal = self.planner_dic['goal']
        rospy.loginfo(f"DEBUG: modified self.goal: {self.goal}")
        # Twist
        self.cmd = geometry_msgs.msg.Twist()

        # TODO BEGIN MRSS: Update the current command
        n_goal = np.linalg.norm(self.goal)
        rospy.loginfo(f"DEBUG: goal norm: {n_goal}")
        if n_goal < 0.1:
            rospy.loginfo(f"DEBUG: goal reached: {self.goal}")
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = 0.
        else:
            # Linear planner commands
            self.cmd.linear.x = self.goal[0]/n_goal * 0.15
            self.cmd.linear.y = self.goal[1]/n_goal * 0.15
            angle_error = np.arctan2(self.goal[1], self.goal[0]) 
            self.cmd.angular.z = np.clip(angle_error, -.2, .2)


    def spin(self):
        """
        Spins the node.
        """
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
        """
        Called on node shutdown.
        """
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
