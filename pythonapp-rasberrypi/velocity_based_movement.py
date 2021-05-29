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
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, #BITMASK ->Consider only the velocities
        0,0,0, #-- POSITION
        vx, vy, vz, #--VELOCITY
        0,0,0, ##-- ACCELERATIONS
        0,0
        )
    vehicle.sendmavlink(msg)
    vehicle.flush()
    
##send a velocity command with +x being TRUE North of the earth
def send_global_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      #time_boot_ms(not used )        
        0,0,    #target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  ##is LOCAL to the earth
        0b0000111111000111, #BITMASK ->Consider only the velocities
        0,0,0, #-- POSITION
        vx, vy, vz, #--VELOCITY in m/s
        0,0,0, ##-- ACCELERATIONS (not supported yet, ignored in GCS_Mavlink ???)
        0,0  ## yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink ???)
        )
    vehicle.sendmavlink(msg)
    vehicle.flush()
        
    
vehicle =connectMyCopter()
vehicle.wait_ready('autopilot_version')

arm_and_takeoff(10)

time.sleep(2)

counter=0
while counter < 5:
	send_local_ned_velocity(5, 0, 0)
	time.sleep(1)
	print("Moving North relative to front of drone")
	counter = counter + 1	
time.sleep(2)
	
counter=0
while counter < 5:
	send_local_ned_velocity(0, -5, 0)
	time.sleep(1)
	print("Moving West relative to front of drone")
	counter = counter + 1

counter=0
while counter < 5:
	send_global_ned_velocity(5, 0, 0)
	time.sleep(1)
	print("Moving TRUE North relative to front of drone")
	counter = counter + 1
time.sleep(2)

counter=0
while counter < 5:
	send_global_ned_velocity(0, -5, 0)
	time.sleep(1)
	print("Moving TRUE West relative to front of drone")
	counter = counter + 1

##UP and DOWN ####
counter=0
while counter < 5:
	send_local_ned_velocity(0, 0, -5)
	time.sleep(1)
	print("Moving UP")
	counter = counter + 1
time.sleep(2)

counter=0
while counter < 5:
	send_global_ned_velocity(0, 0, 5)
	time.sleep(1)
	print("Moving DOWN")
	counter = counter + 1


while True:	
	time.sleep(1)

