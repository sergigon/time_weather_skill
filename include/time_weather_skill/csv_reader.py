import csv
# Tutorial: https://pythonprogramming.net/reading-csv-files-python-3/

# Input: Weather Input, Output: X Json Request

def csv_reader_changer(filename, source, weather_dic_input):
	"""
	Reads a weather csv and returns the info given certain parameters.
	
	@param source: Name of the source to make the request.
	@param weather_dic_input: Input weather dicionary. It must contain forecast and current weather

	@return weather_dic_output: Output standard weather dicionary.
	"""

	# Constants
	FORECAST_TYPE_H = 'forecast_type' # Name of the forecast type header
	WEATHER_INPUT_H = 'weather_input' # Name of the weather input header
	WEATHER_OUTPUT_H = source # Name of the weather output header
	FORECAST_TYPE_CURRENT = 'current' # Name of the weather current list
	FORECAST_TYPE_FORECAST = 'forecast' # Name of the weather forecast list
	FORECAST_TYPE_COMMON = 'common' # Name of the weather common list

	# Variables
	weather_dic_output = {} # Info output

	with open(filename + '.csv') as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')
		
		############ Find csv information ############
		forecast_type_h_col = -1 # Forecast Type column
		weather_input_h_col = -1 # Weather Input column
		weather_output_h_col = -1 #  Weather Output column

		forecast_type_current_row = -1 # Weather Current Type column
		forecast_type_forecast_row = -1 # Weather Forecast Type column
		forecast_type_common_row = -1 # Weather Common Type column
		
		row_len = -1 # Number of rows

		row_n = 0
		for row in csv_reader:
			col_n = 0
			for cell in row:
				# Get weather input and output columns
				if(cell == FORECAST_TYPE_H): # Weather Input column
					forecast_type_h_col = col_n
				if(cell == WEATHER_INPUT_H): # Weather Input column
					weather_input_h_col = col_n
				if(cell == WEATHER_OUTPUT_H): # Json Request Source column
					weather_output_h_col = col_n
				# Get weather types rows
				if(cell == FORECAST_TYPE_CURRENT): # Weather Current row
					forecast_type_current_row = row_n
				if(cell == FORECAST_TYPE_FORECAST): # Weather Forecast Type row
					forecast_type_forecast_row = row_n
				if(cell == FORECAST_TYPE_COMMON): # Weather Common Type row
					forecast_type_common_row = row_n

				col_n = col_n+1
			row_n = row_n+1
		# Get number of rows
		row_len = row_n

		################# Find output ################
		csvfile.seek(0) # Restart the csv iterator

		forecast_type = '' # Actual forecast type
		dic_aux = {}
		row_n = -1
		for row in csv_reader:
			row_n = row_n+1
			# It does not ckeck the headers
			if(row_n == 0):
				continue
			# Check forecast type column
			if(row[forecast_type_h_col] != ''):
				forecast_type = row[forecast_type_h_col]


	return weather_dic_output # Info not found


if __name__ == '__main__':
	# Variables
	filename = 'weather_list'
	forecast_type = 'current' # Forecast Type: 'forecast' or 'current'
	weather_output = 'weather_list' # Json Request Source: 'apixu', ...
	info_required = 'last_updated' # Info Required: 'date', 'temp_c', ...

	info_output = csv_reader_changer(weather_output, filename, 5)
	print("\ninfo_output:")
	print(info_output)