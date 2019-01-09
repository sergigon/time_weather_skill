#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio González Díaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio González Díaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio González Díaz"
__email__ = "serigon@ing.uc3m.es"
__status__ = "Development"

import json
import rospkg
import os
import rospy

class SysOperations():


	# Constructor	
	def __init__(self):
		"""
		Init method.
		"""

	def ls(self, path):
		"""
		Returns list of files in a given directory.
		Tutorial: https://es.stackoverflow.com/questions/24278/c%C3%B3mo-listar-todos-los-archivos-de-una-carpeta-usando-python

		@param path: Full directory path.

		@return list: List of files.
		"""

		return [arch for arch in os.listdir(path) if os.path.isfile(os.path.join(path, arch))]
	
	def ls_json(self, path):
		"""
		Returns list of json files in a given directory.

		@param path: Full directory path.

		@return list_json: List of json files.
		"""

		list_files = self.ls(path)

		list_json = []
		for file in list_files:
			if (file[-5:] == '.json'):
				list_json.append(file)

		return list_json

	def path_exists(self, path):
		"""
		Checks if a path exists.

		@param path: Full directory path.

		@return output: True or false.
		"""

		if os.path.exists(path):
			return True
		else:
			rospy.logwarn("Sys Operations ERROR: File not found (" + path + ")")
			return False

	def write_json(self, filepath, json_dic):
		"""
		Writting method.

		@param filepath: Full file path.
		@param json_dic: JSON dictionary to write.
		"""
		rospy.logdebug('[Sys Operations]: ' + 'Writting ' + filepath)

		# Checks if the file exists
		if not self.path_exists(filepath): # File NOT found
			rospy.logwarn("Creating: " + filepath)

		# Write the data
		with open(filepath, "w") as json_file:
			json.dump(json_dic, json_file)

	def load_json(self, filepath):
		"""
		Loading method.

		@param path: Full file path.

		@return: Return JSON data if file found. If not, return -1.
		"""
		rospy.logdebug('[Sys Operations]: ' + 'Loading' + filepath)

		# Checks if the file exists
		if self.path_exists(filepath): # File found
			# Load the data
			data = json.loads(open(filepath).read())
			return data
		else: # File not found
			return -1

if __name__ == '__main__':
	try:
		print("[sys_operations]: __main__")
		rospy.init_node('my_node', log_level=rospy.DEBUG)

		# rospkg object
		rospack = rospkg.RosPack()

		# Get paths
		pkg_name = "time_weather_skill"
		pkg_path = rospack.get_path(pkg_name) # Package path
		data_path = pkg_path + '/data/' # Data path
		weather_file = data_path + 'prueba.json'

		json_manager = SysOperations()
		#createJson.write_json()
		# Search files
		print(json_manager.ls(data_path))
		print()
		print(json_manager.ls_json(data_path))
		list_json = json_manager.ls_json(data_path)
		print()
		print('prueba_json.json' in list_json)

		# Write json file
		json_manager.write_json({'prueba': 121, 'hola': 'adios'}, weather_file)
		# Load json file
		data = json_manager.load_json(weather_file)
		if(data != -1):
			print('Json content(' + weather_file + '):')
			for key in data:
				print(str(key) + ': ' + str(data[key]))


	except rospy.ROSInterruptException:
		pass

	