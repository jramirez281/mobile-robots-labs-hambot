from carlos56_MyRobot import MyRobot

bot = MyRobot()

#step 1
bot.fwd_lidar_ft(2)
#step 2
bot.fwd_encoder_ft(1, on_carpet=True)
#step 3
bot.fwd_lidar_ft(2)
#step 4
bot.rotate_imu(-180)
#step 5
bot.fwd_lidar_ft(2)
#step 6
bot.fwd_encoder_ft(1, on_carpet=True)
#step 7
bot.fwd_lidar_ft(2, on_carpet=True)
#step 8
bot.rotate_imu(180)
