from robot_systems.robot import HamBot
import time
import math

bot = HamBot(lidar_enabled=False, camera_enabled=False)
#helper function to translate linear distance to revolutions, given radius and linear distance in meters
def linear_to_rev(radius, distance):
    return distance/((2*math.pi)*radius)

## Move forward for 2 seconds
#bot.set_left_motor_speed(-50)   # left motor reversed
#bot.set_right_motor_speed(50)   # right motor forward
#time.sleep(2)
#bot.stop_motors()

wheel_radius = 0.045
axel_radius = 0.205/2
#function to move robot forward x meters
#default speed is .1 m/s
#converting .1 m/s to equivalent value in rpm
default_linear_speed = 0.1
default_speed = ((default_linear_speed/.045)*(60/(2*math.pi)))
def move_forward(distance, speed=default_speed):
    #find the equivalent revolutions first
    revolutions = linear_to_rev(wheel_radius, distance)
    minutes = revolutions/speed
    seconds = minutes*60
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    time.sleep(seconds)

    bot.stop_motors()
       
#turn in place given angle (in degrees), speed can be optionally set as well
def turn_in_place(angle, speed=default_speed):
    turn_distance = (((2*math.pi) * axel_radius)/360) * angle
    turn_distance_rev = linear_to_rev(wheel_radius, turn_distance)
    turn_mins = turn_distance_rev/default_speed
    turn_secs = turn_mins * 60

    bot.set_left_motor_speed(speed * -1)
    bot.set_right_motor_speed(speed)
    time.sleep(turn_secs)

    bot.stop_motors()


