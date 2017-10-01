import json


def sensor_filter(j_sensor, j_osc):
    """
    :param j_sensor: JSON for sensor channel data, each datum is a bit
    :param j_osc: JSON for OSC data, data is signed byte, -127, +127

    :return j_osc: Returns modified j_osc according to the scheme...

    ###SENSOR MAP###

    LEFT<->RIGHT
        TOP
       0 -> 5
    23|------|6
     ^|      ||
     ||      |v
    18|______|11
       17<-12
        BOTTOM

    ##Plain English Algo##
    Given an obstacle in a direction, zero out movement in that direction.
    In the case of any obstacle, zero out turning.
    """

    ##TODO: incorportate "potval" and "toe" timeouterror in channel_data
    potval = j_sensor["potval"]


    #for i in range(25):
    #    if j_sesnor["channel"] == "toe":
    #        pass #figure out what to do with data
    #             #maybe replace with a value

    ##We sum over the sensor values to find if any sensor has
    ##been tripped. Positive non-zero number means obstacle.
    top_obs = sum(j_sensor["channel"][0:5])
    right_obs = sum(j_sensor["channel"][6:11])
    bottom_obs = sum(j_sensor["channel"][12:17])
    left_obs = sum(j_sensor["channel"][18:23])
    any_obs = top_obs + right_obs + bottom_obs + left_obs

    drive = j_osc["drive"] # 127 -> drive forward, -127 -> drive reverse
    if top_obs > 0 & drive > 0:
        drive = 0
    if bottom_obs > 0 & drive < 0:
        drive = 0
    j_osc["drive"] = drive

    strafe = j_osc["strafe"] # 127 -> stafe right, -127 -> strafe left
    if right_obs > 0 & strafe > 0:
        strafe = 0
    if left_obs > 0 & strafe < 0:
        strafe = 0
    j_osc["strafe"] = strafe

    turn = j_osc["turn"] # 127 -> rotate clock-wise, -127 rotate counter-clock-wise
    if any_obs > 0:
        turn = 0

    return j_osc
