import rospy
import pid


class TwistController(object):

    def __init__(self, max_steer_angle):
        ms = max_steer_angle        
	self.steer_pid = pid.PID(kp=0.25, ki=0.006, kd=0.75,mn=-ms, mx=ms)                
        self.timestamp = rospy.get_time()

    def control(self, cte, dbw_enabled):
        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp
        sample_time = duration + 1e-6  

        self.timestamp = new_timestamp
        if dbw_enabled:            
            steering_angle = self.steer_pid.step(cte, sample_time)
            return steering_angle        
        self.steer_pid.reset()

        return 0.0

