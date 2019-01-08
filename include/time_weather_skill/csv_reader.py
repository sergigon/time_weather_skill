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

rospack = rospkg.RosPack() # rospkg object to get the json path

pkg_name = "time_weather_skill"

def csv_reader(filename, source, forecast_type):
	"""
	Reads a weather csv and returns the info given certain parameters.
	
	@param source: Name of the source to get the info.
	@param forecast_type: 'forecast' or 'current'.

	@return url: Url string.
	@return params: Params dicionary.
	@return extra_info: Extra_info dicionary.
	"""

	data_path = rospack.get_path(pkg_name) + '/data/'+ filename + ".csv"


	# Constants
	SOURCE_H = 'source' # Name of the source header
	FORECAST_TYPE_H = 'forecast_type' # Name of the forecast type header
	URL_H = 'url' # Name of the url header
	PARAMS_H = 'params' # Name of the params header
	EXTRA_INFO_H = 'extra_info' # Name of the extra_info header

	# Variables
	params, extra_info = {}, {} # Info output

	################## Open csv ##################
	with open(data_path) as csvfile:
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

	print('######################################')

	################ Search source ################
	# Searchs the source start and end rows.      #
	###############################################
	found = False
	# From header row to end of matrix
	for i in range(h_row+1, len(csv_matrix[:])):
		print(str(i) + ': ' + csv_matrix[i][source_h_col])

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
		rospy.logwarn("csv reader ERROR: Source not found")
		return '', {}, {}

	print('Start of ' + '\'' + source + '\'' + ' found: row ' + str(source_row_start))
	print('End of ' + '\'' + source + '\'' + ' found: row ' + str(source_row_stop))
	print('######################################')

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
			print(str(i) + ': ' + csv_matrix[i][forecast_type_h_col])

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

	print('Start of ' + '\'' + forecast_type + '\'' + ' found: row ' + str(forecast_type_row_start))
	print('End of ' + '\'' + forecast_type + '\'' + ' found: row ' + str(forecast_type_row_stop))
	print('######################################')

	################# Search url ##################
	url = csv_matrix[forecast_type_row_start][url_h_col]
	print('url: ' + url)

	################ Search params ################
	params = {}
	for i in range(forecast_type_row_start, forecast_type_row_stop):
		params.update({csv_matrix[i][params_h_col]: csv_matrix[i][params_h_col+1]})
	print('params: ', params)

	############## Search extra info ##############
	extra_info = {}
	for i in range(forecast_type_row_start, forecast_type_row_stop):
		extra_info.update({csv_matrix[i][extra_info_h_col]: csv_matrix[i][extra_info_h_col+1]})
	print('extra_info: ', extra_info)

	return url, params, extra_info # Info not found


if __name__ == '__main__':
	rospy.init_node('my_node', log_level=rospy.DEBUG)
	# Variables
	filename = 'weather_sources_params'
	source = 'apixu'
	forecast_type = 'current'

	print("INPUT: " + source + ', ' + forecast_type)
	print(csv_reader(filename, source, forecast_type))
