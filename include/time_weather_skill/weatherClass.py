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

from time_weather_skill.create_json import CreateJson
from time_weather_skill.weather_format_changer import source2standard

pkg_name = 'time_weather_skill'

class Weather():

    """
    Weather class.
    """

    _GOAL_MAX_SIZE = 5 # Max size of the goal

    def __init__(self):
        """
        Init method.
        """

        # Weather sources
        self._SOURCE_LIST = ['apixu', 'source1', 'source2'] # List of the sources
        self._create_json = CreateJson() # CreateJson object

    def _save_json(self, json_info, extra_dic):
        """
        Save Json data in a Json file. It adds extra information.

        IMPORTANT: Json data must be in STANDARD format.

        @param json_info: Weather dictionary in STANDARD format.
        @param extra_dic: Dictionary with extra info to save.
        """
        """
        data_folder = '' # Folder to store the json files
        file_name = 'es' + '_' + json_info['common']['city_name'] + '_' + json_info['common']['country_name']

        # Separate the last updated date in various parts 
        date_now = json_info['common']['last_updated'].split(" ") # Get date
        year, month, day = date_now[0].split("-") # Separate date
        """
        # Add extra info
        json_info.update(extra_dic)
 
        # Write the data in the JSON file
        self._create_json.write(json_info, 'prueba_json') # Write weather info into JSON file

    def _URL_request(self, url_forecast, params):
        """
        URL request method.

        @param params: Dictionary with the URL equest params.
        
        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Make URL request
        try:
            r = requests.get(url_forecast, params = params)
            # Get the data from the URL in JSON format
            if 'error' in r.json(): # ERROR in the request
                print('URL request ERROR')
                print(r.json()['error']['message']) # Print the error
                return -1, {}
            else: # NO error in the request
                print('URL request succeded')
                return 0, r.json()
        except:
            print('Connection ERROR')
            return -1, {}

    def _get_weather(self, location, forecast_type, date, info_required):
        """
        Get weather class.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, Spain'.
        @param forecast_type: 'forecast' or 'current'
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize reulsts
        result, result_info_dic = -1, {}
        lang = 'es'

        ############### Checks if it is in local #################
        '''
        if True:
            return 0, result_info_dic
        '''

        ################## Make URL requests #####################
        # >> Each source must get the FORECAST and CURRENT json <<
        # >> info and SAVE it in the 'result_info_dic' variable <<
        for source in self._SOURCE_LIST: # Selects the source from the source list
            print("Making URL request to: '" + source + "'")
            ####### Apixu source #######
            if(source == 'apixu'):
                # Params
                url_forecast = 'http://api.apixu.com/v1/forecast.json'
                params = {
                    'key': '9838870ce6324daf95d161656180811',
                    'q': location,
                    'days': 7, # Forecast range
                    'lang': lang
                    }
                # Request
                result, result_info_dic = self._URL_request(url_forecast, params)

            ####### Source 2 #######
            elif(source == 'source2'):
                #Params
                #Request
                pass

            ####### Request success #######
            if(result == 0): # URL request success
                result_info_dic = source2standard(source, result_info_dic) # Change weather format to standard format
                extra_dic = { # Saves extra info in the file
                    'lang': lang
                }
                self._save_json(result_info_dic, extra_dic) # Saves the content in a local file
                break # Stops the URL requests list


        return result, result_info_dic

    def _manage_weather(self, goal_vec):
        """
        Manager of the weather class. It updates the result and result_info.
        Examples:
        madrid/forecast_type/tomorrow/basic/101
        sydney/current/0/is_day/011

        @param goal_vec: Vector of the goal.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        if(len(goal_vec) >= self._GOAL_MAX_SIZE-1): # Check if all min fields are completed

            ############## Check weather ################
            location = goal_vec[0] # City name
            forecast_type = goal_vec[1] # Forecast or current info
            date = goal_vec[2] # Date
            info_required = goal_vec[3] # Info wanted

            result, result_info_dic = self._get_weather(location, forecast_type, date, info_required) # Get the weather info
            return result, result_info_dic

        else:
            print("Goal size not completed")
            self._result.result = -1 # Fail
            self._result_info_dic = {}
            return -1, {}


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")

        weather = Weather()
        print weather._manage_weather(['madrid', 'forecast', '0', 'date'])

    except rospy.ROSInterruptException:
        pass
