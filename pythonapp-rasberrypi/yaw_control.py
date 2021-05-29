from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
import time
import socket
import exceptions
import math
import argparse #To import some values from command line and use it on our python script
from pymavlink import mavutil


#####################functions####
def connectMyCopter():
	parser=argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect') ##Sec
	args = parser.parse_args()

	connection_string=args.connect
	
	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle=connect(connection_string,wait_ready=True) #To connect the vehicle using ip address and wait ready means it will pass command only when the connection is set up.

	return vehicle
	
def arm_and_takeoff(targetHeight):
	while vehicle.is_armable != True:
		print ("Waiting for vehicle to become armable")
		time.sleep(1)
		
	print("Vehicle is now armable.")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode != "GUIDED":
		print("wainting for drone to enter GUIDED mode ")
		time.sleep(1)

	print("Vehicle now in GUIDED mode.")

	vehicle.armed = True
	while vehicle.armed == False:
		print ("Waiting for vehicle to become armed")
		time.sleep(1)
		
	print("Look out! props are spinning!!")

    vehicle.simple_takeoff(targetHeight) ##meters
	while True:
		print("Current altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= .95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached !!")
	return None
    
##send a velocity command with +x being the heading of the drone
def condition_yaw(degrees, relative):
    if relative:
        is_relative = 1 # Yaw relative to direction of travel
    else:
        is_relative = 0 # yaw is an absolute angle
    #create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  #target system, target components
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  #command
        0, #confirmation
        degrees, #param 1, yaw in degrees
        0, #param 2, yaw speed deg/s
        1, #param 3, direction -1 ccw, 1 cw
        is_relative, #param 4 relative offset 1, absolute angle 0
        0,0, 0 #param 5 - 7 not used
        )
        # send command to vehicle
    vehicle.sendmavlink(msg)
    vehicle.flush()

def dummy_yaw_initializer():
    lat=vehicle.location.global_relative_frame.lat
    lon=vehicle.location.global_relative_frame.lon
    alt=vehicle.location.global_relative_frame.alt
    aLocation=LocationGlobalRelative(lat,lon,alt)

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,      #time_boot_ms(not used )        
        0,0,    #target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  ##Frame
        0b0000111111111000, #BITMASK ->Consider only the speeds enable
        aLocation.lat*1e7,  #lat_int - X Position in WGS84 frame in 1e7*meters
        aLocation.lon*1e7,  #lon_int - Y Position in WGS84 frame in 1e7*meters
        aLocation.alt, #alt - altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT        
        vx, vy, vz, #--VELOCITY in m/s  in NED Frame
        0,0,0, ##-- afx, afy, afz ACCELERATIONS (not supported yet, ignored in GCS_Mavlink ???)
        0,0  ## yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink ???)
        )
        
        # send command to vehicle
    vehicle.sendmavlink(msg)
    vehicle.flush()

        
    
vehicle =connectMyCopter()
vehicle.wait_ready('autopilot_version')

arm_and_takeoff(10)


dummy_yaw_initializer()
time.sleep(2)

condition_yaw(30,1) ##180 -->> 210
print(" Yawing 30 degrees relative to current position ")
time.sleep(7)

print(" Yawing True North ")
condition_yaw(0,0) ##Yaw to true north
time.sleep(7)

print(" Yawing True West ")
condition_yaw(270,0) ##Yaw to true west

while True:	
	time.sleep(1)

