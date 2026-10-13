#experimental controller
from robot_systems.robot import HamBot
import time
import math

class PIDController:
    def __init__(self, kp, ki, kd, dt, lo, hi):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.lo = lo
        self.hi = hi
        self.integral = 0
        self.prev_error = None
        
    def imuUpdate(self, error):
        self.integral += error * self.dt
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / self.dt
        else:
            derivative = 0
            
        raw_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        clamped_output = abs(min(max(self.lo, raw_output), self.hi))
        if raw_output != clamped_output:
            self.integral -= error * self.dt
        
        self.prev_error = error
        return clamped_output
        
    def update(self, error):
        self.integral += error * self.dt
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / self.dt 
        else:
            derivative = 0

        raw_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        clamped_output = min(max(self.lo, raw_output), self.hi)
        if raw_output != clamped_output:
            self.integral -= error * self.dt

        self.prev_error = error
        return clamped_output

class MyRobot(HamBot):
    def __init__(self):
        super().__init__(lidar_enabled=True)
        self.wheel_radius = 0.045
        self.axel_radius = 0.205/2
        self.default_linear_speed = 0.2
        self.dt = 0.05

    #saturation function
    def saturation(self, phi):
        pass

    #helper function to translate linear distance to revolutions, given radius and linear distance in meters
    def linear_to_rev(self, radius, distance):
        return distance/((2*math.pi)*radius)
    #helper functions convert speed in m/s to equivalent value in rpm
    def m_s_to_rpm(self, speed):
        return ((speed/0.045)*(60/(2*math.pi)))
    #helper function to return calibration status
    def get_calibration_status(self):
        return self.imu.sensor.calibration_status

    #default speed is .2 m/s

    #function to gradually accelerate to target speed, to avoid
    #abnormal behavior from jumping to speed right away
    def ramp_speed(self, target, current, step=5):
        if target > current:
            return (current + min(step, target - current))
        elif target < current:
            return (current - min(step, current - target))
        return target

    #function to move robot forward x meters
    def move_forward(self, distance, speed=None):
        if speed is None:
            speed = self.default_linear_speed

        speed_rpm = self.m_s_to_rpm(speed)
        #find the equivalent revolutions first
        revolutions = self.linear_to_rev(self.wheel_radius, distance)
        minutes = revolutions/speed_rpm
        seconds = minutes*60
        self.set_left_motor_speed(speed_rpm)
        self.set_right_motor_speed(speed_rpm)
        time.sleep(seconds)

        self.stop_motors()

    #forward distance pid controlled motion primitive with encoder feedback
    #takes distance to be traveled as parameter
    def forward_encoder(self, goal):
        kp = 100
        ki = 0
        kd = 0
        pid = PIDController(kp, ki, kd, self.dt, -50, 50)
        current = 0
        start = time.time()
        self.reset_encoders()

        while True:
            left_rad = self.get_left_encoder_reading()
            right_rad = self.get_right_encoder_reading()
            left_dist = left_rad * self.wheel_radius
            right_dist = right_rad * self.wheel_radius
            traveled = (left_dist + right_dist) / 2
            error = goal - traveled
            desired = pid.update(error)
            

    #forward distance pid controlled motion primitive with lidar feedback
    #takes goal distance from wall as parameter
    def forward_lidar(self, goal):
        pid = PIDController(kp = 1, ki = 0, kd = 0, dt = 0.032, lo = -15, hi = 15)

        while robot.experiment_supervisor.step(robot.timestep) != -1:
            distance = lidar_range_image()[180]
            error = distance - goal
    
            clampOutput = pid.update(error)
    
            robot.set_left_motor_velocity(clampOutput)
            robot.set_right_motor_velocity(clampOutput)

    def rotation_PID(self, goalPose):
        pid = PIDController(kp = 1, ki = 0, kd = 0, dt = 0.032, lo=-15, hi = 15)

        setPoint = (self.get_compass_reading() + 90) % 360
        
        while robot.experiment_supervisor.step(robot.timestep) != -1:
            currentPose = self.get_compass_reading()
            error = ((setpoint - currentPose + 180) % 360) - 180

            clampOutput = pid.imuUpdate(error)

            if error > 0:
                robot.set_left_motor_velocity(-clampOutput)
                robot.set_right_motor_velocity(clampOutput)
            elif error < 0:
                robot.set_left_motor_velocity(clampOutput)
                robot.set_right_motor_velocity(-clampOutput)

    #turn in place given angle (in degrees), speed can be optionally set as well
    def turn_in_place(self, angle, speed=None):
        if speed is None:
            speed = self.default_linear_speed

        speed_rpm = self.m_s_to_rpm(speed)
        turn_distance = (((2*math.pi) * self.axel_radius)/360) * abs(angle)
        turn_distance_rev = self.linear_to_rev(self.wheel_radius, turn_distance)
        turn_mins = turn_distance_rev/speed_rpm
        turn_secs = turn_mins * 60

        if angle < 0:
            self.set_left_motor_speed(speed_rpm)
            self.set_right_motor_speed(speed_rpm * -1)
        else:
            self.set_left_motor_speed(speed_rpm * -1)
            self.set_right_motor_speed(speed_rpm)
        time.sleep(turn_secs)

        self.stop_motors()

    #move in counter-clockwise curved arc given arc radius (meters) and optional speed
    def travel_arc_ccw(self, radius, speed=None):
        if speed is None:
            speed = self.default_linear_speed

        speed_rpm = self.m_s_to_rpm(speed)
        #distance and time right wheel travels at default speed
        rw_travel_distance = (2*math.pi)*(radius+self.axel_radius)
        rw_revs = self.linear_to_rev(self.wheel_radius, rw_travel_distance)
        travel_mins = rw_revs/speed_rpm
        travel_secs = travel_mins * 60

        #computing speed at which left wheel travels
        lw_travel_distance = (2*math.pi)*(radius-self.axel_radius)
        lw_revs = self.linear_to_rev(self.wheel_radius, lw_travel_distance)
        lw_speed = lw_revs/travel_mins

        self.set_left_motor_speed(lw_speed)
        self.set_right_motor_speed(speed_rpm)
        time.sleep(travel_secs)

        self.stop_motors()

    #move in clockwise curved arc given arc radius (meters) and optional speed
    def travel_arc_cw(self, radius, speed=None):
        if speed is None:
            speed = self.default_linear_speed

        speed_rpm = self.m_s_to_rpm(speed)
        #distance and time left wheel travels at default speed
        lw_travel_distance = (2*math.pi)*(radius+self.axel_radius)
        lw_revs = self.linear_to_rev(self.wheel_radius, lw_travel_distance)
        travel_mins = lw_revs/speed_rpm
        travel_secs = travel_mins * 60

        #computing speed at which right wheel travels
        rw_travel_distance = (2*math.pi)*(radius-self.axel_radius)
        rw_revs = self.linear_to_rev(self.wheel_radius, rw_travel_distance)
        rw_speed = rw_revs/travel_mins

        self.set_right_motor_speed(rw_speed)
        self.set_left_motor_speed(speed_rpm)
        time.sleep(travel_secs)

        self.stop_motors()
