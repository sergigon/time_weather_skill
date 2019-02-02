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

import rospy

import requests # URL requests
import commands # Comandos de terminal (http://www.rafalinux.com/?p=1613)
import datetime # To get actual date

from time_weather_skill.create_json import CreateJson
from time_weather_skill.create_txt import CreateTxt

pkg_name = 'time_weather_skill'

class Apixu():
	"""
    Apixu weather class.
    """

	# Constant names
	apixu_limit_weather = 7 # 7 days limit
	apixu_month_limit_calls = 10000 # 10000 limit calls per month
	api_key_serg = '9838870ce6324daf95d161656180811' # sergigon@ing.uc3m.es
	location_def = "Madrid, Spain" # Se puede especificar pais o no

	# URLs
	url_current = 'http://api.apixu.com/v1/current.json' # URL para current weather
	url_forecast = 'http://api.apixu.com/v1/forecast.json' # URL para forecast weather
	url_icon = 'http://www.apixu.com/doc/Apixu_weather_conditions.json' # URL para informacion de los iconos

	# Lists for the info required
	info_basic_list = ['last_updated', 'city_name', 'country_name', 'date', 'avg_temp_c', 'text', 'code'] # Basic list
	info_advanced_list = info_basic_list[:]
	info_advanced_list.extend(['mintemp_c', 'maxtemp_c', 'totalprecip_mm']) # Advanced list

	# Lists for current request
	update_hours = [23, 19, 14, 9, 4]

	def __init__(self):
		"""
        Init method.
        """

        # Class variables
		self.__info = '' # Weather info
		self.__city_name = '' # Name of the actual city
		self.__result = -1 # Result
		self.__result_info = {} # Result info
		self.__data = '' # Reuest data in JSON format

		# Create JSON
		self.__create_json = CreateJson() # CreateJson object
		self.__file_name = "apixu" # Apixu json and txt file
		self.__file_name_check = self.__file_name + "_check" # Apixu json and txt file

	def _request_URL(self, location, days, lang, key):
		"""
        Request URL method. Modify self.__data variable.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param days: Forecast days.
        @param lang: Language. By default in spanish.
        @param key: Key to make the request.

        @return: True: URL request succeded. False: URL request error.
        """

		# Parametros para hacer el URL request
		params = {
			'key': key,
			'q': location,
			'days': days, # Only for forecast
			'lang': lang # Espanol by default
		}

		# Hago el URL request para el weather
		try:
			r = requests.get(self.url_forecast, params = params)
			# Get the data from the URL in JSON format
			if 'error' in r.json(): # ERROR in the request
				self.__data = {}
				print('URL request ERROR')
				print(r.json()['error']['message']) # Print the error
				return False
			else: # NO error in the request
				self.__data = r.json()
				print('URL request succeded')
				return True
		except:
			print('Connection ERROR')
			return False

		

	def _request_local(self, location, forecast, lang, file_name, file_name_check):
		"""
        Request local method. Modify self.__data variable.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param lang: Language. By default in spanish.
        @param file_name: Name of the file to do the request.
		@param file_name_check: Name of the check file to do the request.

        @return: True: local request succeded. False: local request error.
        """

		# Check if JSON files exists
		data_json = self.__create_json.load(file_name) # Load JSON file
		data_json_check = self.__create_json.load(file_name_check) # Load JSON check file

		make_req = False # Indicate to fill the data
		if (data_json != -1 and data_json_check != -1): # JSON files exists
			# Check all parameters exist
			#try:
			# Check if 'city' and 'lang' coincide
			if(location.lower() == data_json_check['city'].lower() and lang.lower() == data_json_check['lang'].lower()): # 'city' and 'lang' coincide
				now = datetime.datetime.now() # Get current date
				# Check if date is updated
				if(data_json_check['day'] == str(now.day) and data_json_check['month'] == str(now.month) and data_json_check['year'] == str(now.year)): # Already updated
					print('Info in local found: "' + data_json_check['city'].lower() + '", "' + data_json_check['lang'].lower() + '"')
					# Current is selected
					if (forecast == 'current'):
						print('Checking if current weather is updated...')
						hour = -1 # Hour used to update
						for update_hour in self.update_hours: # Search the hour in the list
							hour = update_hour
							if(update_hour < now.hour): # When find the hour it stops
								break
						# Check if current weather is updated
						if(data_json_check['hour'] == str(hour)): # Already updated
							print('Current weather already updated')
							make_req = True
						else:
							print('Current weather NOT updated')
					else:
						print('hola')
						make_req = True
				else:
					rospy.logwarn("Local request ERROR: File not updated")
			else:
				rospy.logwarn("Local request ERROR: 'city' or 'lang' do not coincide")
			#except:
			#	rospy.logwarn("Local request ERROR: Parameters not existing")
		else:
			rospy.logwarn("Local request ERROR: JSON files do not exist")

		if(make_req):
			self.__data = data_json # Get JSON data
			print('Local request succeded')
			return True
		else:
			print('Local request not available')
			return False


	def _request(self, location, forecast, days=apixu_limit_weather, lang='es', key=api_key_serg):
		"""
        Request method.

        Make a local or URL request, and fill the variable self.__data with the info requested.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param days: Forecast days.
        @param lang: Language. By default in spanish.
        @param key: Key to make the request.

        @return: True: request success. False: request error.
        """

		# Initialize the name files
		file_name_full = self.__file_name + '_' + location.lower() + '_' + lang
		file_name_check_full = self.__file_name_check + '_' + location.lower() + '_' + lang
		
		############## Local request ###############
		print("#### Making a local request ####")
		local_request = self._request_local(location, forecast, lang, file_name_full, file_name_check_full)
		if(local_request == True):
			return True
		############################################

		############### URL request ################
		print("#### Making an URL request ####")
		# Get the data from the URL in JSON format
		url_request = self._request_URL(location, days, lang, key) # URL request
		############################################

		
		if (url_request == False): # Error URL request
			print('Error en el URL request')
			print('Not writing JSON file')
			return False

		else: # NO error URL request
			######## Save data in JSON files #######
			# Separate the last updated date in various parts 
			date_now = self.__data['current']['last_updated'].split(" ")
			date_now_vec = date_now[0].split("-")

			# Fill the JSON check file
			data_check = {'last_updated': self.__data['current']['last_updated'],
				'city': self.__data['location']['name'],
				'country': self.__data['location']['country'],
				'lang': lang,
				'day': date_now_vec[2],
				'month': date_now_vec[1],
				'year': date_now_vec[0]
				}
			# Write the data in the JSON files
			self.__create_json.write(self.__data, file_name_full) # Write weather info into JSON file
			self.__create_json.write(data_check, file_name_check_full) # Write weather check info into JSON file
			########################################
			return True

	def _fix_date(self, date):
		"""
		Fix the date and converts to int.

		@param date: date to fix.
		"""

		if (date == 'today'):
			date = '0'
		elif (date == 'tomorrow'):
			date = '1'

		date = int(date) # Converts to int

		return date


	def _get_info(self, date,forecast, info_required):
		"""
		Get info method.

		@param date: date of the forecast.
		@param info_required: type of info requested.

		@return self.__result_info: info requested.
		"""

		# Reset result info
		self.__result_info = {}

		# Converts to int the date for the result_info dictionary
		date = self._fix_date(date)

		# Prepare info_required list
		info_required = info_required.replace(' ','') # Remove spaces form the string
		info_required_list = info_required.split(",") # Separate the goal in various parts

		if('advanced' in info_required_list): # Check if advanced is requested
			info_required_list = self.info_advanced_list[:] # Replace the list with the advanced list

		if('basic' in info_required_list): # Check if basic is requested
			info_required_list = self.info_basic_list[:] # Replace the list with the basic list
		
		# Forecast
		for info_required_i in info_required_list: # Fill the tne result_info variable with the info

			if (info_required_i == 'last_updated'): # last updated
				self.__result_info.update({'last_updated': self.__data['current']['last_updated']})

			if (info_required_i == 'city_name'): # city name
				self.__result_info.update({'city_name': self.__data['location']['name']})

			if (info_required_i == 'country_name'): # country name
				self.__result_info.update({'country_name': self.__data['location']['country']})

			if (info_required_i == 'date'): # date
				self.__result_info.update({'date': self.__data['forecast']['forecastday'][date]['date']})

			if (info_required_i == 'avg_temp_c'): # avg_temp_c
				self.__result_info.update({'avg_temp_c': self.__data['forecast']['forecastday'][date]['day']['avgtemp_c']})

			if (info_required_i == 'text'): # text
				self.__result_info.update({'text': self.__data['forecast']['forecastday'][date]['day']['condition']['text']})

			if (info_required_i == 'code'): # code
				self.__result_info.update({'code': self.__data['forecast']['forecastday'][date]['day']['condition']['code']})

			if (info_required_i == 'mintemp_c'): # mintemp_c
				self.__result_info.update({'mintemp_c': self.__data['forecast']['forecastday'][date]['day']['mintemp_c']})

			if (info_required_i == 'maxtemp_c'): # maxtemp_c
				self.__result_info.update({'maxtemp_c': self.__data['forecast']['forecastday'][date]['day']['maxtemp_c']})

			if (info_required_i == 'totalprecip_mm'): # totalprecip_mm
				self.__result_info.update({'totalprecip_mm': self.__data['forecast']['forecastday'][date]['day']['totalprecip_mm']})

		self.__result = 0

		return self.__result_info

	def _return_result(self):
		"""
		Return self.__result variable stored
		"""

		return self.__result

	def _return_info(self):
		"""
		Return self.__result_info variable stored
		"""

		return self.__result_info
		

if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "]: __main__")

    	apixu = Apixu()
    	apixu._request('Madrid')
    	#apixu._get_info('tomorrow', 'advanced, s')
    	#print(apixu._return_info())

    except rospy.ROSInterruptException:
        pass