
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
from pymavlink import mavutil
from functions import is_tank_full, is_tank_empty, get_distance_metres

vehicle = connect('127.0.0.1:14550', wait_ready=True)






# FUNCTIONS

def arm_vehicle():

    while not vehicle.is_armable:
        print('initialising vehicle')

    vehicle.armed = True

    while not vehicle.armed:
        print('arming vehicle')


def readmission(filename):
    mission_list = []

    with open(filename) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not a relevant waypoint file')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                mission_list.append(cmd)

    return mission_list

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    missionitem = vehicle.commands[0] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    targetWaypointLocation = LocationGlobalRelative(lat,lon)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def is_at_home():
    lat = home.lat
    lon = home.lon
    targetlocation = LocationGlobalRelative(lat, lon)
    distance = get_distance_metres(vehicle.location.global_frame, targetlocation)
    while distance > 10:
        distance = get_distance_metres(vehicle.location.global_frame, targetlocation)
        print('Heading to Launch point, %s meters away' % distance)
        time.sleep(2)
    return True

def is_at_waypoint():
    distance = distance_to_current_waypoint()
    while distance > 10:
        distance = distance_to_current_waypoint()
        print('Heading to waypoint %s, %s meters away' % (waypoint_counter, distance))
        time.sleep(2)
    return True









# MAIN CODE

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print " Waiting for home location ..."

home = vehicle.home_location

arm_vehicle()

mission_list = readmission('waypoints/waypoint_3.waypoints')

waypoint_counter = 1

for command in mission_list:
    if mission_list.index(command) > 0:  # ignore the first command, which is the home location

        counter = 0  # Gives how many back and forth trips to a single waypoint
        while counter < 1:

            cmds = vehicle.commands
            cmds.clear()
            cmds.add(command)
            cmds.upload()
            time.sleep(2)

            vehicle.mode = VehicleMode('AUTO')
            time.sleep(2)

            if is_at_waypoint():
                print('waypoint reached')

                # Turn on relay for conveyor belt
                cmds.download()
                cmds.wait_ready()
                cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0, 0, 2, 1, 0, 0, 0, 0, 0))
                cmds.upload()
                print ('relay turned on')
                time.sleep(2)

                # Once the conveyor belt is on, check to see if the tank is full.
                """
                if is_tank_full():
                    cmds.download()
                    cmds.wait_ready()
                    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0, 0, 2, 0, 0, 0, 0, 0, 0))   # turn off conveyor belt
                    cmds.upload()
"""
                vehicle.mode = VehicleMode('RTL')

            if is_at_home():
                print('Launch point reached')
                # Check if the tank is empty to head back
                # if is_tank_empty():
                #     print ('Heading back')

                time.sleep(2)
            counter += 1
        waypoint_counter += 1


vehicle.close()

