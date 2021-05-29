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
    
    
def get_distance_meters(targetLocation,currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
    
def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
    vehicle.simple_goto(targetLocation)
    
    while vehicle.modo.name == "GUIDED":
        currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
        if currentDistance < distanceToTargetLocation*0.01:
            print ("Reached target waypoint")
            time.sleep(2)
            break
        time.sleep(1)
    return None


        

###>> pythonconnection_template.py --connect 127.0.0.1:14550

#main execution#

wp1 = LocationGlobalRelative(44.50202, -88.060316, 10)
vehicle =connectMyCopter()
vehicle.wait_ready('autopilot_version')

arm_and_takeoff(10)

goto(wp1)

vehicle.mode = VehicleMode("LAND")
while vehicle.mode != "LAND":
    print("wainting for drone to enter LAND mode ")
    time.sleep(1)
print("Vehicle in LAND mode")

#This will keep QGC alive
while True:
    time.sleep(1)

 ##This run like:  launchSitl location_based_movement.py
 
	





