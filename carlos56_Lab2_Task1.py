#Lab 2 Task 1 Code
#Authored by Josueh Ramirez (carlos56), Nathaniel Leblanc (nleblanc)
#CDA4621 F26
import time
from carlos56_MyRobot import MyRobot

bot = MyRobot()

#step 1
bot.fwd_lidar_ft(2)
time.sleep(1)
#step 2
bot.fwd_encoder_ft(1, on_carpet=True)
time.sleep(1)
#step 3
bot.fwd_lidar_ft(2)
time.sleep(1)
#step 4
bot.rotate_imu(-180)
time.sleep(1)
#step 5
bot.fwd_lidar_ft(2)
time.sleep(1)
#step 6
bot.fwd_encoder_ft(1, on_carpet=True)
time.sleep(1)
#step 7
bot.fwd_lidar_ft(2)
time.sleep(1)
#step 8
bot.rotate_imu(180)
time.sleep(1)
#step 9
bot.fwd_encoder_ft(2, on_carpet=True)
