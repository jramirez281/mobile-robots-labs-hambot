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
        self.carpet_scale = 0.91   # measured on carpet at home: 1 ft and 3 ft runs
        self.lidar_offset = 0.097   # m, lidar center to front edge (measured)

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

    #returns a wheel-speed trim (rpm) that keeps the robot on its starting heading
    #while driving straight. Positive trim = robot has drifted clockwise, so the
    #right wheel is sped up and the left slowed (left = speed - trim, right = speed + trim).
    #kh is the gain in rpm per degree of drift; the trim is capped at +-10 rpm.
    #added because the robot veered on carpet when driving in one direction
    def heading_trim(self, hold, kh=1.0):
        heading = self.get_heading(blocking=True)
        drift = ((hold - heading + 180) % 360) - 180
        return max(-10, min(10, kh * drift))

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
    #note: Encoder distance on carpet varied by about ±0.5 in between identical runs (wheel slip). 
    #calibrated with carpet_scale = 0.91 from 1 ft and 3 ft runs.
    def forward_encoder(self, goal, kp, ki, kd):
        pid = PIDController(kp, ki, kd, self.dt, -50, 50)
        current = 0
        start = time.time()
        self.reset_encoders()
        hold = self.get_heading(blocking=True)

        try:
            while True:
                left_rad = self.get_left_encoder_reading()
                right_rad = self.get_right_encoder_reading()
                left_dist = left_rad * self.wheel_radius
                right_dist = right_rad * self.wheel_radius
                traveled = (left_dist + right_dist) / 2
                error = goal - traveled
                desired = pid.update(error) 
                current = self.ramp_speed(desired, current)
                trim = self.heading_trim(hold)
                self.set_left_motor_speed(current - trim)
                self.set_right_motor_speed(current + trim)
                if abs(error) < 0.01 or time.time() - start > 15:
                    print(traveled, error)
                    break
                time.sleep(self.dt)
        finally:
            self.stop_motors()
            
    #wrapper for forward_encoder that takes feet
    #set on_carpet to True when running on carpet, since the thick pile 
    #makes the wheels travel less than the encoders report
    #to be set to false otherwise
    #Default gain constants meant to be final for lab 2
    def fwd_encoder_ft(self, goal, on_carpet, kp=250, ki=5, kd=2):
        goal_meters = goal * 0.3048
        
        if on_carpet:
            goal_meters = goal_meters / self.carpet_scale

        self.forward_encoder(goal_meters, kp, ki, kd)
    
    #forward distance pid controlled motion primitive with lidar feedback
    #takes goal distance from wall as parameter
    #takes distance in meters and converts to mm
    def forward_lidar(self, goal, kp, ki, kd):
        goal_mm = goal * 1000
        pid = PIDController(kp, ki, kd, self.dt, -50, 50)
        current = 0
        start = time.time()
        hold = self.get_heading(blocking=True)

        try:
            while True:
                distance = self.get_range_image()[180]
                if distance <= 0:
                    time.sleep(self.dt)
                    continue
                error = distance - goal_mm
                desired = pid.update(error) 
                current = self.ramp_speed(desired, current)
                trim = self.heading_trim(hold)
                self.set_left_motor_speed(current - trim)
                self.set_right_motor_speed(current + trim)
                if abs(error) < 10 or time.time() - start > 15:
                    print(distance, error)
                    break
                time.sleep(self.dt)
        finally:
            self.stop_motors()

    #wrapper for forward_lidar that takes feet
    #Default gain constants meant to be final for lab 2
    def fwd_lidar_ft(self, goal, kp=0.25, ki=0.005, kd=0.02):
        goal_meters = goal * 0.3048 + self.lidar_offset #for accurate measurement from front of bot
        self.forward_lidar(goal_meters, kp, ki, kd)

    #positive angle = counterclockwise
    #negative angle = clockwise
    #Default gain constants meant to be final for lab 2
    def rotate_imu(self, angle, kp=1, ki=0.05, kd=0.1):
        pid = PIDController(kp, ki, kd, self.dt, -40, 40)

        speed = 0
        start = time.time()

        last = self.get_heading(blocking=True)
        turned = 0
        
        try:
            while True:
                heading = self.get_heading(blocking=True)
                step = ((heading - last + 180) % 360) - 180
                turned += step
                last = heading
                error = angle - turned

                desired = pid.update(error)
                speed = self.ramp_speed(desired, speed)

                self.set_left_motor_speed(-speed)
                self.set_right_motor_speed(speed)
                if abs(error) < 0.5 or time.time() - start > 15:
                    print(heading, error)
                    break
                time.sleep(self.dt)
        finally:
            self.stop_motors()

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
