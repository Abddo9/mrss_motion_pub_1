import numpy
import rospy

class PotentialFieldPlanner():
    def __init__(self, pos_end, dt, k_att, k_rep, vel_max, dist_min=1):
        self.pos_end = numpy.array(pos_end)
        self.dt = dt
        self.k_att = k_att
        self.k_rep = k_rep
        self.vel_max = vel_max
        self.dist_min = dist_min
        self.pos_obs = None
		        
    def set_target_pos (self, end_pos):
        self.pos_end = end_pos
        
    def set_obstacle_distance(self, in_dist_min):
        self.dist_min = in_dist_min

    def set_obstacle_position(self, in_pos_obs):
        if in_pos_obs is None:
            self.pos_obs = None
        else:
            self.pos_obs = numpy.array(in_pos_obs)  
                
    def get_desired_pos_vel(self, pos_fbk):
        _, vel_des = self.get_attractive_force ( pos_fbk )
        pos_des = pos_fbk + vel_des * self.dt 
        
        return pos_des, vel_des
        
    def get_attractive_force ( self, pos_fbk ):
        vel_des = numpy.matmul( self.k_att, ( self.pos_end - pos_fbk ) )
      	
      	# normalize it if the norm is too large
        d = numpy.linalg.norm(vel_des)
        if d > self.vel_max:
            vel_des = vel_des / d * self.vel_max
        pos_des = pos_fbk + vel_des * self.dt
        return pos_des, vel_des
      	
    def get_avoidance_force(self, pos_fbk):
        pos_fbk = numpy.array(pos_fbk)
        _, vel_att = self.get_attractive_force(pos_fbk) 

        # Get repulsive force only if obstacle exists
        if self.pos_obs is not None:
            vel_rep = self.get_repulsive_force(pos_fbk) 
        else:
            vel_rep = numpy.array([0.] * 3)
        
        vel_des = vel_att + vel_rep 
        	   
        # normalize it if the norm is too large
        d = numpy.linalg.norm(vel_des)
        if d > self.vel_max:
            vel_des = vel_des / d * self.vel_max
        pos_des = pos_fbk + vel_des * self.dt
        return pos_des, vel_des
    	
    def get_repulsive_force(self, pos_fbk):
        print("self.pos_obs=", self.pos_obs, "pos_fbk=", pos_fbk )
        rospy.loginfo(self.pos_obs)
        rospy.loginfo(pos_fbk)
        d = numpy.linalg.norm( self.pos_obs[:2] - pos_fbk[:2] )
        
        if d > self.dist_min: # Far enough away, ignore the obstacle
            return numpy.array ([0,0,0])
        else:
            dd_dq = 2 * (self.pos_obs - pos_fbk)
            vel_des = -self.k_rep / (d*d) * (1/d - 1/self.dist_min) * dd_dq
            vel_des[2] = 0.0
            
            # normalize it if the norm is too large
            d = numpy.linalg.norm(vel_des)
            if d > self.vel_max:
                vel_des = vel_des / d * self.vel_max

            return vel_des
