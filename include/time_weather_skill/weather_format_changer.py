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
import rospkg
import datetime
import pytz
from tzwhere import tzwhere
from time_weather_skill.datetime_manager import DatetimeManager
from time_weather_skill.csv_reader import *

LOCAL_COUNTRY = 'es'
LOCAL_TIMEZONE = 'Europe/Madrid'
# datetime.datetime.fromtimestamp(1547575200, tz=pytz.timezone('Europe/Madrid'))

def source2standard(source, forecast_type, weather_dic):
	"""
	Changes the weather dictionary format into the STANDARD format.

	@param source: Name of the source of the request.
	@param weather_dic: Input weather dicionary.

	@return standard_weather_dic: Standard output weather dictionary.
	"""

	# Filenames
	_COUNTRY_CODES_FILENAME = 'wikipedia-iso-country-codes.csv'

	# Paths
	rospack = rospkg.RosPack()
	pkg_name = "time_weather_skill"
	pkg_path = rospack.get_path(pkg_name) + '/' # Package path
	data_path = pkg_path + 'data/' # Data path
	params_path = data_path + 'params/' # Params path

	# File paths
	country_codes_pathfile = params_path + _COUNTRY_CODES_FILENAME

	# Datetime variables
	epoch = datetime.datetime(1970,1,1, tzinfo=pytz.utc) # Epoch in UTC
	now_utc = datetime.datetime.now(tz = pytz.utc) # Actual datetime in UTC
	###########################################################
	############## HOW TO IMPLEMNT NEW SOURCE #################
	# Fill the the standard list variables with the           #
	# corresponding values of each source dictionary.         #
	###########################################################
	# IMPORTANT: If needed, change the content of the         #
	# variable to fit the correct format.                     #
	###########################################################

	################# CHANGE CODE HERE ###################
	# \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ #

	# Initialize variable lists
	f_date, f_avgtemp_c, f_mintemp_c, f_maxtemp_c, f_totalprecip_mm, f_text, f_code, f_icon = [], [], [], [], [], [], [], []

	############################################# Apixu #############################################
	if(source == 'apixu'):
		######### Change forecast type name #########
		# As apixu requests give 'current' and      #
		# 'forecast' at same time, we save both     #
		# info in the standard_weather_dic variable #
		forecast_type = 'all'
		#############################################

		# ########## Current weather ########## #
		c_date = DatetimeManager(weather_dic['location']['localtime']).date() 
		c_temp_c = weather_dic['current']['temp_c']
		c_is_day = weather_dic['current']['is_day']
		c_precip_mm = weather_dic['current']['precip_mm']
		c_text = weather_dic['current']['condition']['text'].lower()
		c_code = weather_dic['current']['condition']['code']
		c_icon = weather_dic['current']['condition']['icon']
		c_last_updated = int((now_utc - epoch).total_seconds())
		# ########## Forecast weather ########## #
		for forecastday in weather_dic['forecast']['forecastday']:
			f_date.append(forecastday['date'])
			f_avgtemp_c.append(forecastday['day']['avgtemp_c'])
			f_mintemp_c.append(forecastday['day']['mintemp_c'])
			f_maxtemp_c.append(forecastday['day']['maxtemp_c'])
			f_totalprecip_mm.append(forecastday['day']['totalprecip_mm'])
			f_text.append(forecastday['day']['condition']['text'].lower())
			f_code.append(forecastday['day']['condition']['code'])
			f_icon.append(forecastday['day']['condition']['icon'])
		f_last_updated = int((now_utc - epoch).total_seconds())
		f_forecast_days = len(weather_dic['forecast']['forecastday'])

		# ########## Common parameters ########## #
		city_name = weather_dic['location']['name']
		code_aux = csv_reader_IO(country_codes_pathfile, 'English short name lower case', weather_dic['location']['country'], 'Alpha-2 code') # Transforms the country name into country code
		country_code = code_aux if (code_aux != -1) else weather_dic['location']['country']
	#################################################################################################

	######################################### OpenWeatherMap ########################################
	elif(source == 'openweathermap'):
		# ########## Current weather ########## #
		if(forecast_type == 'current'):
			# Calculate timezone
			lat, lon = weather_dic['coord']['lat'], weather_dic['coord']['lon']
			timezone = tzwhere.tzwhere().tzNameAt(lat, lon)
			# Actual datetime
			nowtz = datetime.datetime.now(tz = pytz.timezone(timezone)) # Actual datetime in timezone
			print nowtz.hour
			c_date = str(nowtz.year) + '-' + str(nowtz.month) + '-' + str(nowtz.day)
			c_temp_c = weather_dic['main']['temp']
			c_is_day = 1 if (weather_dic['sys']['sunrise'] < weather_dic['dt'] < weather_dic['sys']['sunset']) else 0
			if ('rain' in weather_dic):
				c_precip_mm = weather_dic['rain']['3h'] if ('3h' in weather_dic['rain']) else 0
				c_precip_mm = weather_dic['rain']['1h'] if ('1h' in weather_dic['rain']) else 0
			else:
				c_precip_mm = 0
			c_text = weather_dic['weather'][0]['description'].lower()
			c_code = weather_dic['weather'][0]['id']
			c_icon = 'http://openweathermap.org/img/w/' + str(weather_dic['weather'][0]['icon']) + '.png'
			c_last_updated =  int((now_utc - epoch).total_seconds())
			city_name = weather_dic['name']
			country_code = weather_dic['sys']['country']
			
		# ########## Forecast weather ########## #
		if(forecast_type == 'forecast'):
			# Calculate timezone
			lat, lon = weather_dic['city']['coord']['lat'], weather_dic['city']['coord']['lon']
			timezone = tzwhere.tzwhere().tzNameAt(lat, lon)
			# Actual datetime
			nowtz = datetime.datetime.now(tz = pytz.timezone(timezone)) # Actual datetime in timezone
			FORECAST_HOUR = 16 # Hour at which the info is taken (Local time)

			######## List the indexes of each forecast day #########
			# days = [dayIndexes1, dayIndexes2, ...] #
			days = [] # List to store each day list
			dayIndexes = [] # List to store the indexes of one day

			# Transforms the datetime into the datetime of the timezone
			dtForecastdaytz = datetime.datetime.fromtimestamp(weather_dic['list'][0]['dt'], tz=pytz.timezone(timezone)) # Datetime in timezone
			# If first forecastday is different from the actual day, for the actual day it takes the prevision of that datetime
			if(nowtz.day != dtForecastdaytz.day):
				dayIndexes.append(0)
				days.append(dayIndexes)
				f_date.append(str(nowtz.year) + '-' + str(nowtz.month) + '-' + str(nowtz.day))
				dayIndexes = []
			# First date in the list
			actualDate = dtForecastdaytz.date()
			i=-1
			for forecastday in weather_dic['list']:
				i+=1
				# Transforms the datetime into the datetime of the timezone
				dtForecastdaytz = datetime.datetime.fromtimestamp(forecastday['dt'], tz=pytz.timezone(timezone)) # Datetime in timezone
				# If there is a day of difference between actual and previous date it fills the days list and initialize the indexes list and 
				if(dtForecastdaytz.day != actualDate.day):
					print dtForecastdaytz.day, actualDate.day
					days.append(dayIndexes)
					f_date.append(str(actualDate.year) + '-' + str(actualDate.month) + '-' + str(actualDate.day))
					dayIndexes = []
					actualDate = dtForecastdaytz.date() # Update actualDate with actual date
				# Append actual index
				dayIndexes.append(i)
			# Append last index date list
			days.append(dayIndexes)
			f_date.append(str(actualDate.year) + '-' + str(actualDate.month) + '-' + str(actualDate.day))
			########################################################
			
			for dayIndexes in days:
				###################### Day i #######################
				mintempList = []
				maxtempList = []
				avgtemp = 0
				forecastHour = True # Variable that indicates if the hour forecast must be realized
				for index in dayIndexes:
					# Transforms the datetime into the datetime of the timezone
					dtForecastdaytz = datetime.datetime.fromtimestamp(weather_dic['list'][index]['dt'], tz=pytz.timezone(timezone)) # Datetime in timezone
					print dtForecastdaytz.time()
					# It is the forecast hour or the last hour
					if((dtForecastdaytz.hour > FORECAST_HOUR or index == dayIndexes[-1]) and forecastHour == True):
						totalprecip_mm = weather_dic['list'][index]['rain']['3h'] if ('rain' in weather_dic['list'][index] and '3h' in weather_dic['list'][index]['rain']) else 0
						text = weather_dic['list'][index]['weather'][0]['description'].lower()
						code = weather_dic['list'][index]['weather'][0]['id']
						icon ='http://openweathermap.org/img/w/' + str(weather_dic['list'][index]['weather'][0]['icon']) + '.png'
						forecastHour = False

					# Entire day variables
					avgtemp += weather_dic['list'][index]['main']['temp']
					mintempList.append(weather_dic['list'][index]['main']['temp_min'])
					maxtempList.append(weather_dic['list'][index]['main']['temp_max'])
				
				# Forecast hour
				f_avgtemp_c.append(round(avgtemp/len(dayIndexes), 2))
				f_mintemp_c.append(min(mintempList))
				f_maxtemp_c.append(max(maxtempList))
				# Entire day
				f_totalprecip_mm.append(totalprecip_mm)
				f_text.append(text)
				f_code.append(code)
				f_icon.append(icon)
				####################################################

			f_last_updated = int((now_utc - epoch).total_seconds())
			f_forecast_days = len(days)
			city_name = weather_dic['city']['name']
			country_code = weather_dic['city']['country']
	#################################################################################################

	########################################### No source ###########################################
	else:
		rospy.logerr('[Weather Format Changer] ' + '\'' + source + '\' not found')
		return -1
	#################################################################################################

	# /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ #
	######################################################



	########### Standard dictionary list #############
	##################################################
	# DATA MUST HAVE THE FORMAT SPECIFIED            #
	##################################################
	# ONLY MODIFY THE CODE TO CHANGE OR ADD          #
	# NEW PARAMETRES TO THE STANDARD LIST            #
	##################################################
	# MORE INFO IN README.MD                         #
	##################################################
	standard_weather_dic = {}
	#==============# Current weather #===============#
	if(forecast_type == 'current' or forecast_type == 'all'): # 'Currrent' or 'all' data selected
		current = {
			'date': c_date, # '2018-11-08' TIMEZONE datetime text
			'temp_c': c_temp_c, # 51.8
			'is_day': c_is_day, # 1
			'precip_mm': c_precip_mm, # 0.2
			'text': c_text, # 'Partly cloudy'
			'code': c_code, # 1003
			'icon': c_icon, # '//cdn.apixu.com/weather/64x64/day/266.png'
			'last_updated': c_last_updated # '1541695515' UTC datetime epoch
		}
		standard_weather_dic.update({'current': current}) # Saves data in standard_weather_dic
	#=============# Forecast weather #===============#
	if(forecast_type == 'forecast' or forecast_type == 'all'): # 'Forecast' or 'all' data selected
		# Forecast day list
		forecastday = []
		i=0
		for x in f_date:
			forecastday.append({
					'date': f_date[i], # '2018-11-08' TIMEZONE datetime text
					'avgtemp_c': f_avgtemp_c[i], # 10.7
					'mintemp_c': f_mintemp_c[i], # 8.3
					'maxtemp_c': f_maxtemp_c[i], # 12.9
					'totalprecip_mm': f_totalprecip_mm[i], # 0.04
					'text': f_text[i], # 1240
					'code': f_code[i], # 'Light rain shower'
					'icon': f_icon[i], # '//cdn.apixu.com/weather/64x64/day/266.png'
				})
			i+=1
		# Forecast weather
		forecast = {
			'forecastday': forecastday,
			'last_updated': f_last_updated, # '1541695515' UTC datetime epoch
			'forecast_days': f_forecast_days # 7
			}
		standard_weather_dic.update({'forecast': forecast}) # Saves data in standard_weather_dic
	#=============# Common parameters #==============#
	common = {
		'city_name': city_name, # 'Madrid'
		'country_code': country_code, # 'ES'
		'source': source # 'apixu'
	}
	standard_weather_dic.update({'common': common}) # Saves data in standard_weather_dic
	##################################################

	return standard_weather_dic

if __name__ == '__main__':

	# Functioning example
	apixu_dic = {
		u'current': {
			u'precip_mm': 0.2,
			u'last_updated': u'2018-11-08 17:45',
			u'wind_degree': 50,
			u'wind_kph': 6.1,
			u'is_day': 1,
			u'uv': 4.0,
			u'temp_f': 51.8,
			u'vis_miles': 4.0,
			u'temp_c': 11.0,
			u'humidity': 94,
			u'last_updated_epoch': 1541695515,
			u'cloud': 75,
			u'feelslike_c': 10.6,
			u'wind_mph': 3.8,
			u'feelslike_f': 51.1,
			u'wind_dir': u'NE',
			u'pressure_mb': 1013.0,
			u'vis_km': 8.0,
			u'precip_in': 0.01,
			u'pressure_in': 30.4,
			u'condition': {
				u'text': u'Partly cloudy',
				u'code': 1003,
				u'icon': u'//cdn.apixu.com/weather/64x64/day/116.png'
			}
		},
		u'location': {
			u'name': u'Madrid',
			u'country': u'Spain',
			u'region': u'Madrid',
			u'tz_id': u'Europe/Madrid',
			u'lon': -3.68,
			u'lat': 40.4,
			u'localtime_epoch': 1541695960,
			u'localtime': u'2018-11-08 17:52'
		},
		u'forecast': {
			u'forecastday': [{
				u'date': u'2018-11-08',
				u'astro': {
					u'moonrise': u'08:25 AM',
					u'moonset': u'07:00 PM',
					u'sunset': u'06:04 PM',
					u'sunrise': u'07:53 AM'
				},
				u'date_epoch': 1541635200,
				u'day': {
					u'avgvis_miles': 10.0,
					u'avghumidity': 75.0,
					u'totalprecip_mm': 1.1,
					u'avgtemp_c': 10.7,
					u'uv': 1.9,
					u'avgtemp_f': 51.2,
					u'maxwind_mph': 12.5,
					u'mintemp_f': 46.9,
					u'maxtemp_c': 12.9,
					u'mintemp_c': 8.3,
					u'maxtemp_f': 55.2,
					u'totalprecip_in': 0.04,
					u'maxwind_kph': 20.2,
					u'avgvis_km': 16.4,
					u'condition': {
						u'text': u'Light rain shower',
						u'code': 1240,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/353.png'
					}
				}
			},
			{
				u'date': u'2018-11-09',
				u'astro': {
					u'moonrise': u'09:29 AM',
					u'moonset': u'07:39 PM',
					u'sunset': u'06:03 PM',
					u'sunrise': u'07:54 AM'
				},
				u'date_epoch': 1541721600,
				u'day': {
					u'avgvis_miles': 11.0,
					u'avghumidity': 62.0,
					u'totalprecip_mm': 0.7, 
					u'avgtemp_c': 9.9,
					u'uv': 2.6,
					u'avgtemp_f': 49.9,
					u'maxwind_mph': 13.2,
					u'mintemp_f': 48.9,
					u'maxtemp_c': 13.2,
					u'mintemp_c': 9.4,
					u'maxtemp_f': 55.8,
					u'totalprecip_in': 0.03,
					u'maxwind_kph': 21.2, 
					u'avgvis_km': 18.3, 
					u'condition': {
						u'text': u'Moderate rain at times',
						u'code': 1186,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/299.png'
					}
				}
			},
			{
				u'date': u'2018-11-10',
				u'astro': {
					u'moonrise': u'10:29 AM',
					u'moonset': u'08:21 PM',
					u'sunset': u'06:02 PM',
					u'sunrise': u'07:55 AM'
				},
				u'date_epoch': 1541808000,
				u'day': {
					u'avgvis_miles': 10.0,
					u'avghumidity': 86.0,
					u'totalprecip_mm': 1.7,
					u'avgtemp_c': 9.4,
					u'uv': 0.9,
					u'avgtemp_f': 49.0,
					u'maxwind_mph': 17.9,
					u'mintemp_f': 38.7,
					u'maxtemp_c': 11.7,
					u'mintemp_c': 3.7,
					u'maxtemp_f': 53.1,
					u'totalprecip_in': 0.07,
					u'maxwind_kph': 28.8,
					u'avgvis_km': 17.1,
					u'condition': {
						u'text': u'Light drizzle',
						u'code': 1153,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/266.png'
					}
				}
			},
			{
				u'date': u'2018-11-11',
				u'astro': {
					u'moonrise': u'11:25 AM',
					u'moonset': u'09:08 PM',
					u'sunset': u'06:01 PM',
					u'sunrise': u'07:56 AM'
				},
				u'date_epoch': 1541894400,
				u'day': {
					u'avgvis_miles': 10.0,
					u'avghumidity': 83.0,
					u'totalprecip_mm': 2.3,
					u'avgtemp_c': 13.6,
					u'uv': 1.4,
					u'avgtemp_f': 56.5,
					u'maxwind_mph': 12.8,
					u'mintemp_f': 52.5,
					u'maxtemp_c': 15.7,
					u'mintemp_c': 11.4,
					u'maxtemp_f': 60.3,
					u'totalprecip_in': 0.09,
					u'maxwind_kph': 20.5,
					u'avgvis_km': 16.2,
					u'condition': {
						u'text': u'Light drizzle',
						u'code': 1153,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/266.png'
					}
				}
			},
			{
				u'date': u'2018-11-12',
				u'astro': {
					u'moonrise': u'12:15 PM',
					u'moonset': u'09:58 PM',
					u'sunset': u'06:00 PM',
					u'sunrise': u'07:57 AM'
				},
				u'date_epoch': 1541980800, 
				u'day': {
					u'avgvis_miles': 11.0,
					u'avghumidity': 81.0,
					u'totalprecip_mm': 1.1,
					u'avgtemp_c': 12.3,
					u'uv': 0.4,
					u'avgtemp_f': 54.1,
					u'maxwind_mph': 8.7,
					u'mintemp_f': 51.3,
					u'maxtemp_c': 13.8,
					u'mintemp_c': 10.7,
					u'maxtemp_f': 56.8,
					u'totalprecip_in': 0.04,
					u'maxwind_kph': 14.0,
					u'avgvis_km': 18.0,
					u'condition': {
						u'text': u'Patchy rain possible',
						u'code': 1063,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/176.png'
					}
				}
			},
			{
				u'date': u'2018-11-13',
				u'astro': {
					u'moonrise': u'01:00 PM',
					u'moonset': u'10:52 PM',
					u'sunset': u'05:59 PM',
					u'sunrise': u'07:58 AM'
				},
				u'date_epoch': 1542067200,
				u'day': {
					u'avgvis_miles': 12.0,
					u'avghumidity': 64.0,
					u'totalprecip_mm': 0.1,
					u'avgtemp_c': 14.7,
					u'uv': 39960.0,
					u'avgtemp_f': 58.5,
					u'maxwind_mph': 5.1,
					u'mintemp_f': 53.6,
					u'maxtemp_c': 21.1,
					u'mintemp_c': 12.0,
					u'maxtemp_f': 70.0,
					u'totalprecip_in': 0.0,
					u'maxwind_kph': 8.3,
					u'avgvis_km': 19.4,
					u'condition': {
						u'text': u'Patchy rain possible',
						u'code': 1063,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/176.png'
					}
				}
			},
			{
				u'date': u'2018-11-14',
				u'astro': {
					u'moonrise': u'01:40 PM',
					u'moonset': u'11:48 PM',
					u'sunset': u'05:58 PM',
					u'sunrise': u'07:59 AM'
				},
				u'date_epoch': 1542153600,
				u'day': {
					u'avgvis_miles': 12.0,
					u'avghumidity': 66.0,
					u'totalprecip_mm': 0.0,
					u'avgtemp_c': 14.3,
					u'uv': 39960.0,
					u'avgtemp_f': 57.8,
					u'maxwind_mph': 8.5,
					u'mintemp_f': 53.8,
					u'maxtemp_c': 18.5,
					u'mintemp_c': 12.1,
					u'maxtemp_f': 65.3,
					u'totalprecip_in': 0.0,
					u'maxwind_kph': 13.7,
					u'avgvis_km': 20.0,
					u'condition': {
						u'text': u'Partly cloudy',
						u'code': 1003,
						u'icon': u'//cdn.apixu.com/weather/64x64/day/116.png'
					}
				}
			}
		]}
	}
	result_info = source2standard('apixu', 'current', apixu_dic)
	print(result_info)