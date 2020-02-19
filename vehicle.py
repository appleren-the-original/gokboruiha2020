import time

import dronekit

class Vehicle:
	def __init__(self, connection_mode, ip, port):
		self.connection_mode = connection_mode
		self.ip = ip
		self.port = port

		self.connection_string = connection_mode + ":" + ip + ":" + str(port)
		print("Dronekit, connecting to: {}".format(self.connection_string))

		self.vehicle = dronekit.connect(self.connection_string, wait_ready=True)
		self.v = self.vehicle

		print("Vehicle is ready")
