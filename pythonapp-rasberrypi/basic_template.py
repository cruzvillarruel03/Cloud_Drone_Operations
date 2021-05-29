from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse #To import some values from command line and use it on our python script

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

###>> pythonconnection_template.py --connect 127.0.0.1:14550

#main execution#
vehicle =connectMyCopter()
vehicle.wait_ready('autopilot_version')

arm_and_takeoff(10)

	




