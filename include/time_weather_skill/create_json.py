#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Elena Velázquez and Sergio González"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Elena Velázquez and Sergio González"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Elena Velázquez"
__email__ = "serigon@ing.uc3m.es"
__status__ = "Development"

import json
import rospkg
import os
import rospy

rospack = rospkg.RosPack() # rospkg object to get the json path

pkg_name = "time_weather_skill"

class CreateJson:

	# Constructor	
	def __init__(self):
		"""
		Init method.
		"""
	
	def file_exists(self, file_name):
		"""
		Checks if a file exists.

		@param file_name: Name of the file, without extension.
		"""

		if os.path.exists(self.get_full_path(file_name)):
			return True
		else:
			return False

	def get_full_path(self, file_name):
		"""
		Returns the full path.
		
		@params file_name: Name of the file, without extension.

		@return: Returns full path.
		"""

		return rospack.get_path(pkg_name) + "/data/" + file_name + ".json"

	def write(self, data, file_name):
		"""
		Writting method.

		@params data: JSON dictionary to write.
		@params file_name: Name of the file, without extension.
		"""
		# Checks if the file exists
		if not self.file_exists(file_name): # File found
			print(self.get_full_path(file_name) + " file not found")
			print("Creating: " + self.get_full_path(file_name))

		# Write the data
		with open(self.get_full_path(file_name), "w") as json_file:
   			json.dump(data, json_file)

   	def load(self, file_name):
   		"""
		Loading method.

		@params file_name: Name of the file, without extension

		@return: Return JSON data if file found. If not, return -1
		"""

   		# Checks if the file exists
   		if self.file_exists(file_name): # File found
   			# Load the data
   			data = json.loads(open(self.get_full_path(file_name)).read())
   			return data
   		else: # File not found
   			print(self.get_full_path(file_name) + " file not found")
   			return -1

if __name__ == '__main__':
	try:
		print("[create_json]: __main__")
		createJson = CreateJson()
		#createJson.write()
		data = createJson.load("data")
		if(data != -1):
			print(data['current']['last_updated'])

	except rospy.ROSInterruptException:
		pass

	