from robot_systems.robot import HamBot
import time
import math

class MyRobot(Hambot):
    def __init__(self):
        super().__init__()
    #helper function to translate linear distance to revolutions, given radius and linear distance in meters
    def linear_to_rev(self, radius, distance):
        return distance/((2*math.pi)*radius)
    def m_s_to_rpm(self, speed):
        return ((speed/0.045)*(60/(2*math.pi)))

    wheel_radius = 0.045
    axel_radius = 0.205/2
    #default speed is .1 m/s
    #helper functions convert speed in m/s to equivalent value in rpm
    default_linear_speed = 0.1

    #function to move robot forward x meters
    def move_forward(self, distance, speed=default_linear_speed):
        speed_rpm = m_s_to_rpm(speed)
        #find the equivalent revolutions first
        revolutions = linear_to_rev(wheel_radius, distance)
        minutes = revolutions/speed_rpm
        seconds = minutes*60
        bot.set_left_motor_speed(speed_rpm)
        bot.set_right_motor_speed(speed_rpm)
        time.sleep(seconds)

        bot.stop_motors()

    #turn in place given angle (in degrees), speed can be optionally set as well
    def turn_in_place(self, angle, speed=default_linear_speed):
        speed_rpm = m_s_to_rpm(speed)
        turn_distance = (((2*math.pi) * axel_radius)/360) * abs(angle)
        turn_distance_rev = linear_to_rev(wheel_radius, turn_distance)
        turn_mins = turn_distance_rev/speed_rpm
        turn_secs = turn_mins * 60

        if angle < 0:
            bot.set_left_motor_speed(speed_rpm)
            bot.set_right_motor_speed(speed_rpm * -1)
        else:
            bot.set_left_motor_speed(speed_rpm * -1)
            bot.set_right_motor_speed(speed_rpm)
        time.sleep(turn_secs)

        bot.stop_motors()

    #helper function to make robot face north
    def face_north(self, speed=default_linear_speed):
        current_heading = bot.get_heading()
        target_heading = 90
        turn_angle = (target_heading-current_heading + 180) % 360 - 180
        turn_in_place(turn_angle, speed=speed)

    #move in counter-clockwise curved arc given arc radius (meters) and optional speed
    def travel_arc_ccw(self, radius, speed=default_linear_speed):
        speed_rpm = m_s_to_rpm(speed)
        #distance and time right wheel travels at default speed
        rw_travel_distance = (2*math.pi)*(radius+axel_radius)
        rw_revs = linear_to_rev(wheel_radius, rw_travel_distance)
        travel_mins = rw_revs/speed_rpm
        travel_secs = travel_mins * 60

        #computing speed at which left wheel travels
        lw_travel_distance = (2*math.pi)*(radius-axel_radius)
        lw_revs = linear_to_rev(wheel_radius, lw_travel_distance)
        lw_speed = lw_revs/travel_mins

        bot.set_left_motor_speed(lw_speed)
        bot.set_right_motor_speed(speed_rpm)
        time.sleep(travel_secs)

        bot.stop_motors()

    #move in clockwise curved arc given arc radius (meters) and optional speed
    def travel_arc_cw(self, radius, speed=default_linear_speed):
        speed_rpm = m_s_to_rpm(speed)
        #distance and time left wheel travels at default speed
        lw_travel_distance = (2*math.pi)*(radius+axel_radius)
        lw_revs = linear_to_rev(wheel_radius, lw_travel_distance)
        travel_mins = lw_revs/speed_rpm
        travel_secs = travel_mins * 60

        #computing speed at which right wheel travels
        rw_travel_distance = (2*math.pi)*(radius-axel_radius)
        rw_revs = linear_to_rev(wheel_radius, rw_travel_distance)
        rw_speed = rw_revs/travel_mins

        bot.set_right_motor_speed(rw_speed)
        bot.set_left_motor_speed(speed_rpm)
        time.sleep(travel_secs)

        bot.stop_motors()

    def drive_rectangle(self, L, W, speed=default_linear_speed):
        move_forward(W/2, speed=speed)
        turn_in_place(-90, speed=speed)
        move_forward(L, speed=speed)
        turn_in_place(-90, speed=speed)
        move_forward(W, speed=speed)
        turn_in_place(-90, speed=speed)
        move_forward(L, speed=speed)
        turn_in_place(-90, speed=speed)
        move_forward(W/2, speed=speed)

    def drive_triangle(self, S, speed=default_linear_speed):
        move_forward(S, speed=speed)
        turn_in_place(120, speed=speed)
        move_forward(S, speed=speed)
        turn_in_place(120, speed=speed)
        move_forward(S, speed=speed)
        turn_in_place(120, speed=speed)

    def lab1_path(self, L, W, R1, R2, S, speed=default_linear_speed):
        #step 1 - Rectangle
        drive_rectangle(L, W, speed=speed)

        #step 2 - Counterclockwise Circle
        travel_arc_ccw(R1, speed=speed)

        #step 3 - Clockwise Circle
        travel_arc_cw(R2, speed=speed)

        #step 4 - Equilateral Triangle
        drive_triangle(S, speed=speed)

    def lab1_demo(self):
        print("Welcome to the Lab 1 demo!")
        print("Enter Rectangle length (in meters): ")
        L = float(input())
        print("Enter Rectangle width (in meters): ")
        W = float(input())
        print("Enter Counterclockwise Circle Radius (in meters): ")
        R1 = float(input())
        print("Enter Clockwise Circle Radius (in meters): ")
        R2 = float(input())
        print("Enter Equilateral Triangle Side Length (in meters): ")
        S = float(input())
        print("Enter speed (in m/s), leave blank to use default speed: ")
        speed = input()
        if speed == "":
            speed = default_linear_speed
        speed = float(speed)
        print("Press Enter to continue.")
        input()

        lab1_path(L, W, R1, R2, S, speed=speed)
