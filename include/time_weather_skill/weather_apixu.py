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

	def __init__(self):
		"""
        Init method.
        """

        # Class variables
		self.__info = '' # Weather info
		self.__city_name = ''
		self.__result = -1
		self.__result_info = ''

	def _request(self, location=location_def, days=apixu_limit_weather, lang='es', key=api_key_serg):
		"""
        Request method.

        @param location: City to calculate weather. Format: "Madrid" or "Madrid, Spain".
        @param days: Forecast days.
        @param lang: Language. By default in spanish
        @param key: Key to make the request
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
		self.__data = r.json() # Get the data from the URL in Json format

		# If error set something #
		#                        #
		##########################

		print(type(self.__data))
		#print(self.__data)

	def _get_info(self, date, info_required):
		"""
		Get info method.

		@param date: date of the forecast.
		@param imfo_required: 
		"""

		date = int(date) # Converts to intthe date for the dictionary
		# Forecast
		if (info_required == 'basic' or info_required == 'advanced'):
			self.__result_info = {
				'last_updated': self.__data['current']['last_updated'], # last updated
				'city name': self.__data['location']['name'], # city name
				'country_name': self.__data['location']['country'], # country name
				'date': self.__data['forecast']['forecastday'][date]['date'], # date
				'avg_temp_c': self.__data['forecast']['forecastday'][date]['day']['avgtemp_c'], # avgtemp_c
				'text': self.__data['forecast']['forecastday'][date]['day']['condition']['text'], # text
				'code': self.__data['forecast']['forecastday'][date]['day']['condition']['code'] # code
			}
			if (info_required == 'advanced'):
				self.__result_info.update({
					'mintemp_c': self.__data['forecast']['forecastday'][date]['day']['mintemp_c'], # mintemp_c
					'maxtemp_c': self.__data['forecast']['forecastday'][date]['day']['maxtemp_c'], # maxtemp_c
					'totalprecip_mm': self.__data['forecast']['forecastday'][date]['day']['totalprecip_mm'] # totalprecip_mm
					})
		print(self.__result_info)

if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")

    	apixu = Apixu()
    	apixu._request('Madrid')
    	apixu._get_info('1', 'basic')

    except rospy.ROSInterruptException:
        pass