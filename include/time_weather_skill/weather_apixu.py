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
			'lang': lang # Espanol
		}

		# Hago el URL request para el weather
		r = requests.get(url_forecast, params = params)
		data = r.json() # Get the data from the URL in Json format
	'''
	# Que tiempo hara la semana que viene
	forecast_day = 6
	#print(data['forecast']['forecastday'][forecast_day])

	print("El proximo dia " + data['forecast']['forecastday'][forecast_day]['date'] + " ")


	######## Showing Icon ##########
	icon_code = '' # Codigo del icono
	icon_number = '' # Numero de la imagen del icono
	day = 'day' # Indica dia o noche

	print(data['forecast']['forecastday'][forecast_day]['day']['condition']['text'])
	print(data['forecast']['forecastday'][forecast_day]['day']['condition']['code'])
	icon_code = data['forecast']['forecastday'][forecast_day]['day']['condition']['code'] # Guardo el codigo del icono

	# Pido informacion sobre los codigos y numeros de los iconos
	i = requests.get(url_icon)
	icons_data = i.json()

	for icon in icons_data:
		if (icon['code'] == icon_code):
			icon_number = str(icon['icon'])
			break

	command = 'fim -a weather_icons/64x64/' + day + '/' + icon_number + '.png'

	res = commands.getstatusoutput(command)
	if res[0] == 0:
		print res[1]
	else:
		print "Error: "+ str(res[0])
		print "Descripcion: " + res[1]
	'''


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")
    except rospy.ROSInterruptException:
        pass