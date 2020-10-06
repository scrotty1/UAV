##!/ usr / bin / env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
Added code for flying using NED Velocity Vectors - Jane Cleland-Huang 1/15
"""
import math
import os
import time
from math import sin, cos, atan2, radians, sqrt

from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from dronekit_sitl import SITL

from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller

def connect_virtual_vehicle(instance, home):
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    instance_arg = '-I%s' %(str(instance))
    home_arg = '--home=%s, %s,%s,180' % (str(home[0]), str(home[1]), str(home[2]))
    speedup_arg = '--speedup=4'
    sitl_args = [instance_arg, '--model', 'quad', home_arg, speedup_arg]
    #sitl_args = [instance_arg, '--model', 'quad', home_arg]
    sitl.launch(sitl_args, await_ready=True)
    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + instance * 10)
    conn_string = ':'.join([tcp, ip, port])
    print('Connecting to vehicle on: %s' % conn_string)

    vehicle = connect(conn_string)
    vehicle.wait_ready(timeout=120)
    print("Reached here")
    return vehicle, sitl


################################################################################################
# ARM and TAKEOFF
################################################################################################

# function:   	arm and takeoff
# parameters: 	target altitude (e.g., 10, 20)
# returns:	n/a

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("home: " + str(vehicle.location.global_relative_frame.lat))

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
            print("Reached target altitude")
            break
        time.sleep(1)


################################################################################################
# function:    Get distance in meters
# parameters:  Two global relative locations
# returns:     Distance in meters
################################################################################################
def get_distance_meters(locationA, locationB):
    # approximate radius of earth in km
    R = 6373.0

    lat1 = radians(locationA.lat)
    lon1 = radians(locationA.lon)
    lat2 = radians(locationB.lat)
    lon2 = radians(locationB.lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    # print("Distance (meters):", distance)
    return distance


################################################################################################
# Fly to a target location using a waypoint
################################################################################################
def fly_to(vehicle, targetLocation, groundspeed):
    print("Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(
        vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon))
    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_meters(currentTargetLocation, vehicle.location.global_frame)
        print("Distance Remaining: " + str(remainingDistance))
        if remainingDistance < 1:
            print("Reached target")
            break
        time.sleep(1)

################################################################################################
# Given the GPS coordinates of the center of a circle, and a degree (0-360) and radius
# return a point on the circumference.
################################################################################################
#def point_on_circle(radius, angle_indegrees, latitude, longitude):
    # Convert from degrees to radians
#    lon = (radius * math.cos(angle_indegrees * math.pi / 180)) + longitude
#    lat = (radius * math.sin(angle_indegrees * math.pi / 180)) + latitude
#    return Location(lat, lon)


############################################################################################
# Main functionality
############################################################################################

vehicle, sitl = connect_virtual_vehicle(1,([41.7144367,-86.2417136,0])) #"41.7144367,-86.2417136,221,0"
start = time.time()

arm_and_takeoff(10)

# Get current location of vehicle and establish a conceptual circle around it for flying
#center = Location(vehicle.location.global_relative_frame.lat,
#                  vehicle.location.global_relative_frame.lon)  
#radius = .0001
#angle = 0  #Starting angle

# Fly to starting position using waypoint
#currentLocation = center
#startingPos = point_on_circle(radius*.95, angle+1, center.lat, center.lon)  # Close to first position on circle perimeter
log1 = CoordinateLogger()

startLat = vehicle.location.global_relative_frame.lat
startLon = vehicle.location.global_relative_frame.lon
startLocation = Location(startLat, startLon)

log1.add_data(startLat, startLon)

print("Hovering for 5 seconds ...")
time.sleep(5)

firstTargetLat = startLat + (14.14213562 * 9.003516864e-6)
firstTargetLon = startLon + (14.14213562 * -1.20163249e-5)
firstTargetPosition = LocationGlobalRelative(firstTargetLat, firstTargetLon, 15)
print("Flying NW for approx 20m and increasing altitude to 15m ...")
fly_to(vehicle, firstTargetPosition, 2)
log1.add_data(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
print("Altitude: " + str(vehicle.location.global_relative_frame.alt))

secondTargetLat = firstTargetLat
secondTargetLon = firstTargetLon + (20 * 1.20163249e-5)
secondTargetPosition = LocationGlobalRelative(secondTargetLat, secondTargetLon, 15)
print("Flying E for approx 20m ...")
fly_to(vehicle, secondTargetPosition, 2)
log1.add_data(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)

finalTargetPosition = LocationGlobalRelative(startLat, startLon, 10)
print("Heading back to original location with altitude of 10m ...")
fly_to(vehicle, finalTargetPosition, 2)
log1.add_data(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)


print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

end = time.time()
print(end - start)

###########################################################################
# Create the target coordinates
###########################################################################
log2 = CoordinateLogger()

###########################################################################
# Plot graph
###########################################################################
plotter = GraphPlotter(log1.lat_array,log1.lon_array,log2.lat_array,log2.lon_array,"Longitude","Latitude","NED vs. target Coordinates")
plotter.scatter_plot()
