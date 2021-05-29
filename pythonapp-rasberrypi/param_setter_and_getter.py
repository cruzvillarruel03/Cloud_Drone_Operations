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

###>> pythonconnection_template.py --connect 127.0.0.1:14550

#main execution#
vehicle =connectMyCopter()

vehicle.wait_ready('autopilot_version')
	
#supports set attitude from pc companion
vehicle.capabilities.set_attitude_target_local_ned

#vehicle.location.global_relative_frame

#vehicle.attitude

#vehicle.etc...etc...etc

gps_type = vehicle.parameters['GPS_TYPE']
vehicle.parameters['GPS_TYPE'] = 3
gps_type = vehicle.parameters['GPS_TYPE']

print('GPS_TYPE param value is %s"%str(gps_type))

vehicle.close()



