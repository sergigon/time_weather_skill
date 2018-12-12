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
		self.__create_text = CreateTxt() # CreateJson object

	def _request_URL(self, location, days=apixu_limit_weather, lang='es', key=api_key_serg):
		"""
        Request URL method.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param days: Forecast days.
        @param lang: Language. By default in spanish.
        @param key: Key to make the request.

        @return: Request URL request data.
        """
		# Parametros para hacer el URL request
		params = {
			'key': key,
			'q': location,
			'days': days, # Only for forecast
			'lang': lang # Espanol by default
		}

		# Hago el URL request para el weather
		r = requests.get(self.url_forecast, params = params)
		# Get the data from the URL in JSON format
		return r.json()
		print('JSON content not correct')

	def _request(self, location, days=apixu_limit_weather, lang='es', key=api_key_serg):
		"""
        Request method.

        Make a local or URL request, and fill the variable self.__data with the info requested.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param days: Forecast days.
        @param lang: Language. By default in spanish.
        @param key: Key to make the request.
        """
		
		############## Local request ###############
		# Check if JSON files exists
		data_json = self.__create_json.load(self.__file_name) # Load JSON file
		data_json_check = self.__create_json.load(self.__file_name_check) # Load JSON check file

		updated = False # Variable to check if already updated
		if (data_json != -1 and data_json_check != -1): # JSON files exists
			try:
				# Check if 'city' and 'lang' coincide
				if(location == data_json_check['city'] and lang == data_json_check['lang']): # 'city' and 'lang' coincide
					now = datetime.datetime.now() # Get current date
					if(data_json_check['day'] == now.day and data_json_check['month'] == now.month and data_json_check['year'] == now.year): # Already updated
						print("Already made request, not making new request")
						self.__data = data_json # Get JSON data
						updated = True
			except:
				print("Parameters not existing")
		else:
			print("JSON files do not exist")
		############################################

		############### URL request ################
		if (updated == False): # NOT updated
			print("Making an URL request")
			# Get the data from the URL in JSON format
			self.__data = self._request_URL(location, days, lang, key) # URL request
			if (self.__data != -1): # NO error URL request
				date_now = self.__data['current']['last_updated'].split(" ") # Separate the goal in various parts
				date_now_vec = date_now[0].split("-")
				# Write the data in the json files
				data_check = {'last_updated': self.__data['current']['last_updated'],
					'city': self.__data['location']['name'],
					'country': self.__data['location']['country'],
					'lang': lang,
					'day': date_now_vec[2],
					'month': date_now_vec[1],
					'year': date_now_vec[0]
					}
				self.__create_json.write(self.__data, self.__file_name) # Write weather info into JSON file
				self.__create_json.write(data_check, self.__file_name_check) # Write weather check info into JSON file
			else: # Error URL request
				print('Error en el request')
			
		############################################
		
		
		

		

		# If error set something #
		#                        #
		##########################
		

	def _update_json(self):
		"""
		Looks if there is already weather info in local.

		Used to not make unnecessary requests.
		"""

		self.__create_json.file_exists(self.__json_file_name)

	def _fix_date(self, date):
		"""
		Fix the date and converts to int.
		"""

		if (date == 'today'):
			date = '0'
		elif (date == 'tomorrow'):
			date = '1'

		date = int(date) # Converts to int

		return date


	def _get_info(self, date, info_required):
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