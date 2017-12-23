
class SpeedController(object):
    def __init__(self, vehicle_mass, wheel_radius, accel_limit,
                 decel_limit, brake_deadband, fuel_capacity,
                 max_acceleration):
        
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband
        
        self.max_acc_torque = self.vehicle_mass * max_acceleration * self.wheel_radius
        
        self.max_brake_torque = self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius


    def control(self, target_velocity, current_velocity, realization_time):
        
        error = target_velocity - current_velocity        
        acceleration = error / realization_time
        
        if acceleration > 0:
            acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)

        throttle, brake = 0., 0.

        if abs(acceleration) < self.brake_deadband:
            return throttle, brake
        
        torque = self.vehicle_mass * acceleration * self.wheel_radius

        if torque > 0:            
            throttle, brake = min(1.0, torque / self.max_acc_torque), 0.0
        else:           
            throttle, brake = 0.0, min(abs(torque), self.max_brake_torque)

        return throttle, brake
