import json


def sensor_filter(j_sensor, j_osc):
    """
    :param j_sensor: JSON for sensor channel data, each datum is a bit
    :param j_osc: JSON for OSC data, data is signed byte, -127, +127

    :return j_osc: Returns modified j_osc according to the scheme...

    ###SENSOR MAP###

    LEFT<->RIGHT
        TOP
       0 -> 6
    23|------|6
     ^|      ||
     ||      |v
    18|______|12
       18<-12
        BOTTOM

    ##Plain English Algo##
    Given an obstacle in a direction, zero out movement in that direction.
    In the case of any obstacle, zero out turning.
    """

    ##TODO: incorportate "potval" and "toe" timeouterror in channel_data
    #potval = j_sensor["potval"]

    threshold = 550
    
    #filter small distance values( which are glitches at large distance)
    for i in range(24):
        if j_sensor["channel_data"][i] < 100:
            j_sensor["channel_data"][i] = 8192

    if j_sensor["channel_data"][22] > 300:
	j_sensor["channel_data"][22] = 600

    for j in range(24):

        #map values between 8190 mm and the threshold to 0
        if j_sensor["channel_data"][j] > threshold:
            j_sensor["channel_data"][j] = 0

        #map values between threshold and small valid values to 1  
        elif j_sensor["channel_data"][j] <= threshold:
            j_sensor["channel_data"][j] = 1

    

     
    #for i in range(25):
    #    if j_sesnor["channel"] == "toe":
    #        pass #figure out what to do with data
    #             #maybe replace with a value

    ##We sum over the sensor values to find if any sensor has
    ##been tripped. Positive non-zero number means obstacle.
    top_obs = sum(j_sensor["channel_data"][0:6])
    right_obs = sum(j_sensor["channel_data"][7:11]) + j_sensor["channel_data"][12]
    bottom_obs = sum(j_sensor["channel_data"][12:16]) + sum(j_sensor["channel_data"][17:18])
    left_obs = sum(j_sensor["channel_data"][18:21]) + sum(j_sensor["channel_data"][22:23])
    any_obs = top_obs + right_obs + bottom_obs + left_obs
    print top_obs
    print right_obs
    print bottom_obs
    print left_obs
    print any_obs

    drive = j_osc["drive"] # 127 -> drive forward, -127 -> drive reverse
    strafe = j_osc["strafe"]
    turn = j_osc["turn"]
    if top_obs > 0 and drive > 0:
        drive = 0
    if bottom_obs > 0 and drive < 0:
        drive = 0
    #j_osc["drive"] = drive

    #strafe = j_osc["strafe"] # 127 -> stafe right, -127 -> strafe left
    if right_obs > 0 and strafe > 0:
        strafe = 0
    if left_obs > 0 and strafe < 0:
        strafe = 0
    #j_osc["strafe"] = strafe

    #turn = j_osc["turn"]
   # if any_obs > 0:
        #turn = 0
    j_osc["drive"] = drive
    j_osc["strafe"] = strafe	
    j_osc["turn"] = turn # 127 -> rotate clock-wise, -127 rotate counter-clock-wise

    return j_osc
