import numpy as np

def rotate_robot(time_step, threshold=500):
    if time_step < threshold:
        ang_vel = 0.2
    else:
        ang_vel = -0.2
    return ang_vel

def align_with_goal(goal_pos):
    angle_error = np.arctan2(goal_pos[1], goal_pos[0]) 
    return np.clip(angle_error, -.2, .2)