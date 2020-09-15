#!/usr/bin/env python
from __future__ import print_function
import math
import random
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from dronekit_sitl import SITL
import time
import argparse

lat_1 = 41715167
lon_1 = -86243146
lat_2 = 41714350
lon_2 = -86243146
speed = 15
rate = speed/5 #pings per second
home_lat = 41.714841
home_long = -86.241941

#Coordinates
right = -86240335
left = -86243146
top = 41715167
bottom = 41714350
box_found = False

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
# This technique for starting SITL allows us to specify defffaults 
if not connection_string:
    sitl_defaults = '~/git/ardupilot/tools/autotest/default_params/copter.parm'
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=41.714841, -86.241941,0,180']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string = 'tcp:127.0.0.1:5760'

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print ('Current position of vehicle is: %s' % vehicle.location.global_frame)


def hide_black_box():
    vertical_range = top-bottom
    horizontal_range = left-right

    # Create latitude
    i=0
    rand_num = random.random()
    vertical_offset = rand_num * vertical_range
    new_latitude = bottom + vertical_offset
    new_latitude = new_latitude/1000000
    new_latitude = "%.6f" % new_latitude

    # Create longitude
    rand_num = random.random()
    horizontal_offset = rand_num * horizontal_range
    new_longitude = (left-horizontal_offset)/1000000
    new_longitude = "%.6f"%(new_longitude)
    print("Black box hidden")
    print("Black Box Latitude is " + new_latitude)
    print("Black Box Longitude is " + new_longitude)
    return (new_latitude, new_longitude, 0)

hidden_coords = hide_black_box()
hidden_spot = LocationGlobalRelative(float(hidden_coords[0]), float(hidden_coords[1]), float(hidden_coords[2]))

def arm_and_takeoff(a_target_altitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(3)
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed!")
    print("Taking off!")
    vehicle.simple_takeoff(a_target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= a_target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(10)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        ping(vehicle.location.global_relative_frame, hidden_spot)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            print(vehicle.location.global_relative_frame)
            break;
        time.sleep(.33)


def ping(alocation1, hidden_spot):
    dist_between = get_distance_metres(alocation1, hidden_spot)
    print("Distance between target: " + dist_between)
    if (dist_between <= 5):
        finish()


def finish():
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = hidden_spot
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(.33)

        # Land when the box is found
        print("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        # Close vehicle object before exiting script
        print("Close vehicle object")
        vehicle.close()
        # Shut down simulator if it was started
        if sitl is not None:
            sitl.stop()



#box_loc = hide_black_box()
#print(box_loc)
# Fly in a snake pattern to try and find box starting at the top left
print("TRIANGLE path using standard Vehicle.simple_goto()")
print("Set groundspeed to 15m/s")
vehicle.groundspeed=15
print("Heading to top right: North 41.715167 West -86.243147")
goto(41.715167, -86.243147)




# Land when the box is found
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
# Shut down simulator if it was started
if sitl is not None:
    sitl.stop()

print("Completed")

