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
    
    
vehicle =connectMyCopter()
vehicle.wait_ready('autopilot_version')

arm_and_takeoff(10)

    

## class dronekit.Command(target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)

##cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
##    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-34.364114, 149.166022, 30)        

wphome = vehicle.location.global_relative_frame
##list of commands
cmd1=Command(0,0,0,mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,wphome.lat, wphome.lon, wphome.alt)
cmd2=Command(0,0,0,mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,44.501375,-88.062645,15)
cmd3=Command(0,0,0,mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,44.501746,-88.062242,10)
cmd4=Command(0,0,0,mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0,)




##download current list of commands from the drone connected
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

##clear the current list of commands
cmds.clear()

##add in out new commands
cmds.add( cmd1 )
cmds.add( cmd2 )
cmds.add( cmd3 )
cmds.add( cmd4 )

##upload our commands to the drone
vehicle.commands.upload()

arm_and_takeoff(10)

print("after arm and takeoff ")

vehicle.mode = VehicleMode("AUTO")

while vehicle.mode != "AUTO":
    print("wainting for drone to enter AUTO mode ")
    time.sleep(0.2)

print("Vehicle now in AUTO mode.")

while vehicle.location.global_relative_frame.alt > 2:
    print ("Drone is executing mission, but we can still run code")
    time.sleep(2)
    
        



