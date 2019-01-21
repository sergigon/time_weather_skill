#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import csv # Tutorial: https://pythonprogramming.net/reading-csv-files-python-3/
import rospkg
import rospy
from time_weather_skill.sys_operations import SysOperations

def csv_reader_params(filepath, source, forecast_type):
	"""
	Reads a weather csv and returns the info given certain parameters.
	
	@param filepath: Full file path.
	@param source: Name of the source to get the info.
	@param forecast_type: 'forecast' or 'current'.

	@return url: Url string.
	@return params: Params dicionary.
	@return extra_info: Extra_info dicionary.
	"""

	rospy.logdebug('[Csv Reader]: Searching source \'' + source + '\' in '+ filepath)

	# If file does not exist
	if(not SysOperations().path_exists(filepath)):
		rospy.logerr("[Csv Reader] Csv Reader Params ERROR: File does not exist")
		return -1, {}, {}

	# Constants
	SOURCE_H = 'source' # Name of the source header
	FORECAST_TYPE_H = 'forecast_type' # Name of the forecast type header
	URL_H = 'url' # Name of the url header
	PARAMS_H = 'params' # Name of the params header
	EXTRA_INFO_H = 'extra_info' # Name of the extra_info header

	# Variables
	params, extra_info = {}, {} # Info output

	################## Open csv ##################
	with open(filepath) as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')
		
		############ Find csv headers ############
		source_h_col = -1 # Source column
		forecast_type_h_col = -1 # Forecast Type column
		url_h_col = -1 # Url column
		params_h_col = -1 # Params column
		extra_info_h_col = -1 # Extra Info column

		row_len = -1 # Number of rows

		h_row = -1 # Row where are located the headers
		source_row_start = -1 # Source row start
		source_row_stop = -1 # Source row stop
		forecast_type_row_start = -1 # Forecast Type row start
		forecast_type_row_stop = -1 # Forecast Type row stop


		csv_matrix = [] # csv matrix variable

		row_n = -1
		for row in csv_reader:
			csv_matrix.append(row) # Write the csv into the matrix
			row_n = row_n+1
			col_n = -1
			for cell in row:
				col_n = col_n+1
				# Get headers columns
				if(cell == SOURCE_H): # Source column
					source_h_col = col_n
					h_row = row_n
				if(cell == FORECAST_TYPE_H): # Forecast Type column
					forecast_type_h_col = col_n
				if(cell == URL_H): # Url column
					url_h_col = col_n
				if(cell == PARAMS_H): # Params column
					params_h_col = col_n
				if(cell == EXTRA_INFO_H): # Extra Info column
					extra_info_h_col = col_n


	################ Search source ################
	# Searchs the source start and end rows.      #
	###############################################
	found = False
	# From header row to end of matrix
	for i in range(h_row+1, len(csv_matrix[:])):
		# Search the source row
		if(csv_matrix[i][source_h_col] == source):
			source_row_start = i
			found = True

		# Searchs the end source row (next source or end of list)
		if(found):
			if(csv_matrix[i][source_h_col] != '' and i != source_row_start): # Next forecast
				source_row_stop = i
				break
			elif(i == len(csv_matrix[:])-1): # End of list
				source_row_stop = i+1
				break

	# Source not found
	if(source_row_start == -1):
		rospy.logerr('[Csv Reader] Csv Reader Params ERROR: Source \'' + source + '\' not found')
		return -1, {}, {}

	############ Search forecast type #############
	# Searchs for the forecast type START         #
	# and END rows.                               #
	###############################################
	forecast_type_row_start = source_row_start
	forecast_type_row_stop = source_row_stop
	found = False
	
	# Check if forecast_type is specified in the csv
	if(csv_matrix[source_row_start][forecast_type_h_col] != ''):
		# From start source row to end source row
		for i in range(source_row_start, source_row_stop):
			# Search the forecast type row
			if(csv_matrix[i][forecast_type_h_col] == forecast_type):
				forecast_type_row_start = i
				found = True

			# Searchs the end forecast type row (next forecast type or end of list)
			if(found):
				if(csv_matrix[i][forecast_type_h_col] != '' and i != forecast_type_row_start): # Next forecast
					forecast_type_row_stop = i
					break
				elif(i == source_row_stop): # End of list
					forecast_type_row_stop = i+1
					break

	################# Search url ##################
	url = csv_matrix[forecast_type_row_start][url_h_col]
	rospy.logdebug('[Csv Reader]: ' + 'url: ' + str(url))

	################ Search params ################
	params = {}
	for i in range(forecast_type_row_start, forecast_type_row_stop):
		params.update({csv_matrix[i][params_h_col]: csv_matrix[i][params_h_col+1]})
	rospy.logdebug('[Csv Reader]: ' + 'params: ' + str(params))

	############## Search extra info ##############
	extra_info = {}
	for i in range(forecast_type_row_start, forecast_type_row_stop):
		extra_info.update({csv_matrix[i][extra_info_h_col]: csv_matrix[i][extra_info_h_col+1]})
	rospy.logdebug('[Csv Reader]: ' + 'extra_info: ' + str(extra_info))

	return url, params, extra_info # Info not found

def csv_reader_country_codes(filepath, headerInput, inputInfo, headerOutput):
	"""
	Reads an standard column csv using the names of the headers and the input to get the output.
	
	@param filepath: Full file path.
	@param headerInput: Name of the hedaer of the info input.
	@param inputInfo: Input value.
	@param headerOutput: Name of the hedaer of the info output.

	@return result: Result of the request.
	"""

	# If file does not exist
	if(not SysOperations().path_exists(filepath)):
		rospy.logerr("[Csv Reader] Csv Reader Country Codes ERROR: File does not exist")
		return -1 # Fail (-1)

	################## Open csv ##################
	with open(filepath) as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')

		############ Find csv headers ############
		headerInput_col = -1 # headerInput column
		headerOutput_col = -1 # headerOutput column

		row_n=-1
		for row in csv_reader:
			row_n+=1
			# Search the headers columns
			if(row_n == 0):
				col_n=-1
				for cell in row:
					col_n+=1
					if(cell == headerInput): # Source Code column
						headerInput_col = col_n
					if(cell == headerOutput): # Info Requested column
						headerOutput_col = col_n
				continue
			# Search the info requested
			if(row[headerInput_col].lower() == str(inputInfo).lower()): # Info found
				rospy.logdebug("[Csv Reader] Info '" + str(inputInfo) + "' from '" + headerInput + "' found: " + str(row[headerOutput_col]))
				return row[headerOutput_col]

	rospy.logwarn("[Csv Reader] Csv Reader Country Codes ERROR: Info '" + str(inputInfo) + "' from '" + headerInput + "' not found")
	return -1

def csv_reader_conditions(filepath, source, source_code, info_requested):
	"""
	Reads a weather csv and returns the info given certain parameters.
	
	@param filepath: Full file path.
	@param source: Name of the source to get the condition code.
	@param source_code: Int source code used.
	@param info_requested: Information requested.

	@return result: Result of the request.
	"""

	# Add '_code' to the source name
	source = source + '_code'
	# Transforms the source code in string to make the search
	source_code = str(source_code)

	# If file does not exist
	if(not SysOperations().path_exists(filepath)):
		rospy.logerr("[Csv Reader] Csv Reader Conditions ERROR: File does not exist")
		return -1 # Fail (-1)

	################## Open csv ##################
	with open(filepath) as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')

		############ Find csv headers ############
		source_code_h_col = -1 # Source Code column
		info_requested_h_col = -1 # Info Requested column

		row_n=-1
		for row in csv_reader:
			row_n+=1
			# Search the headers columns
			if(row_n == 0):
				col_n=-1
				for cell in row:
					col_n+=1
					if(cell == source): # Source Code column
						source_code_h_col = col_n
					if(cell == info_requested): # Info Requested column
						info_requested_h_col = col_n
				continue
			# Search the info requested
			cell = row[source_code_h_col].replace(' ','') # Remove spaces from the string
			cell = cell.split(",") # Separate the string if needed
			for cell_i in cell:
				if (cell_i == source_code): # Code found
					rospy.logdebug("[Csv Reader] Code '" + str(source_code) + "' from '" + source + "' found: " + str(row[info_requested_h_col]))
					return row[info_requested_h_col]

	rospy.logerr("[Csv Reader] Csv Reader Conditions ERROR: Code '" + str(source_code) + "' from '" + source + "' not found")
	return -1

if __name__ == '__main__':
	print("[csv_reader]: __main__")
	rospy.init_node('my_node', log_level=rospy.DEBUG)

	pkg_name = "time_weather_skill"
	# rospkg object
	rospack = rospkg.RosPack()
	# Get paths
	pkg_path = rospack.get_path(pkg_name) # Package path
	data_path = pkg_path + '/data/' # Data path

	#___________csv_reader_params___________
	'''
	# Variables
	filename = 'weather_sources_params'
	source = 'apixu'
	forecast_type = 'current'
	# Get paths
	csv_file = data_path + filename + '.csv'
	# Get info
	print("INPUT: " + source + ', ' + forecast_type)
	print(csv_reader_params(csv_file, source, forecast_type))
	'''

	#___________csv_reader_conditions___________
	# Get paths
	filepath = data_path + 'conditions_codes_standard.csv'
	# Get info
	print(csv_reader_conditions(filepath, 'openweathermap', '503', 'standard_icon'))
	#___________csv_reader_country_codes___________
	filepath = data_path + 'wikipedia-iso-country-codes.csv'
	print(csv_reader_country_codes(filepath, 'Alpha-2 code', 'ES', 'English short name lower case'))
	print(csv_reader_country_codes(filepath, 'English short name lower case', 'Spain', 'Alpha-2 code'))
	