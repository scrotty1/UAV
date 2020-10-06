# Import DroneKit-Python
import math
import os
import time
from math import sin, cos, atan2, radians, sqrt
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative

#Setup option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information')
parser.add_argument('--connect',help="vehicle connection target string.")
args=parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle.
print "\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=False)
vehicle.wait_ready(timeout=120)

#Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print "GPS: %s" % vehicle.gps_0
print "Battery: %s" % vehicle.battery
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Is Armable?: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name # settable

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True 
    print("Vehicle armed!")

    #Takeoff
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    print('Current location after takeoff is: {0},{1},{2}'.format(lat,lon,alt))

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


####################################
# YOUR CODE GOES HERE
####################################
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

############################################################################################
# Main functionality
############################################################################################
# Arm and takeoff to 10m
arm_and_takeoff(10)

# Establishing starting coordinates 
startLat = vehicle.location.global_relative_frame.lat
startLon = vehicle.location.global_relative_frame.lon
startLocation = LocationGlobalRelative(startLat, startLon, vehicle.location.global_relative_frame.alt)

# Hover for 5 seconds
print("Hovering for 5 seconds ...")
time.sleep(5)

# First target is ~20m North West of starting position & altitude increases to 15m
firstTargetLat = startLat + (14.14213562 * 9.003516864e-6) # meters * degreesLat per meter
firstTargetLon = startLon + (14.14213562 * -1.20163249e-5) # meters * degreesLon per meter
firstTargetPosition = LocationGlobalRelative(firstTargetLat, firstTargetLon, 15)
print("Flying NW for approx 20m and increasing altitude to 15m ...")
fly_to(vehicle, firstTargetPosition, 2)
print("Altitude: " + str(vehicle.location.global_relative_frame.alt))

# Second target is ~20m East of current location 
secondTargetLat = firstTargetLat
secondTargetLon = firstTargetLon + (20 * 1.20163249e-5)
secondTargetPosition = LocationGlobalRelative(secondTargetLat, secondTargetLon, 15)
print("Flying E for approx 20m ...")
fly_to(vehicle, secondTargetPosition, 2)

# Final target brings us back to original location and altitude of 10m
finalTargetPosition = LocationGlobalRelative(startLat, startLon, 10)
print("Heading back to original location with altitude of 10m ...")
fly_to(vehicle, finalTargetPosition, 2)


####################################
# Add this at the end
####################################
print("Returning to Launch")
vehicle.mode = VehicleMode("LAND")  # Note we replace RTL with LAND

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Close vehicle object before exiting script
vehicle.close()

time.sleep(5)

print("Completed")

