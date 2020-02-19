from dronekit import LocationGlobal, VehicleMode, LocationGlobalRelative, connect
import time
import math
from pymavlink import mavutil
import pymavlink

def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)

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

	# Wait until the vehicle reaches a safe height before processing the goto
	#  (otherwise the command after Vehicle.simple_takeoff will execute
	#   immediately).
	while True:
		print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		# Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
			print("Reached target altitude")
			break
		time.sleep(1)

def condition_yaw(heading, relative=False):
	"""
	Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

	This method sets an absolute heading by default, but you can set the `relative` parameter
	to `True` to set yaw relative to the current yaw heading.

	By default the yaw of the vehicle will follow the direction of travel. After setting 
	the yaw using this function there is no way to return to the default yaw "follow direction 
	of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

	For more information see: 
	http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	"""
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		1,          # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# send command to vehicle
	vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# send command to vehicle on 1 Hz cycle
	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)


def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned Location has the same `alt` value
	as `original_location`.
	The function is useful when you want to move the vehicle around specifying locations relative to 
	the current vehicle position.
	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius=6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return LocationGlobal(newlat, newlon,original_location.alt)

def go_in_meters(dNorth,dEast):
	vehicle.simple_goto(get_location_metres(vehicle.location.global_frame,dNorth,dEast))

def change_altitude(altitude):
	vehicle.mode=VehicleMode("GUIDED")
	loc=LocationGlobal (vehicle.location.global_frame.lat , vehicle.location.global_frame.lon , altitude)
	vehicle.simple_goto(loc)