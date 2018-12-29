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
from time_weather_skill.csv_reader import csv_reader_changer

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

    def _format_change(self, source, json_data):
        """
        Changes the weather dictionary format into the STANDARD format.
        
        @param source: Name of the source to make the request.
        @param json_data: Input weather dicionary.

        @return standard_weather_dic: Standard output weather dictionary.
        """

        info_output = csv_reader_changer(filename, source, json_data)
        print("info_output: '" + info_output + "'")

        pass


    def _save_json(self, json_info):
        """
        Save Json data in a Json file. It adds extra information.

        IMPORTANT: Json data must be in STANDARD format.

        @param json_info: Weather dictionary in STANDARD format.
        """
        """
        data_folder = '' # Folder to store the json files
        file_name = 'es' + '_' + json_info['common']['city_name'] + '_' + json_info['common']['country_name']

        # Separate the last updated date in various parts 
        date_now = json_info['common']['last_updated'].split(" ") # Get date
        year, month, day = date_now[0].split("-") # Separate date
        """
        # Add extra info
        json_info.update({'lang': 'es'})
        #json_info.update({'day': day})
        #json_info.update({'month': month})
        #json_info.update({'year': year})
 
        # Write the data in the JSON file
        self._create_json.write(json_info, 'prueba_json') # Write weather info into JSON file



    def _URL_request(self, source, location):
        """
        URL request method.

        @param source: Name of the source to make the request.
        @param location: City to get the weather. Format: "Madrid" or "Madrid, Spain".
        
        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Update URL and parameters to make the URL request
        url_forecast = ''
        params = {}
        if(source == 'apixu'): # Apixu source
            url_forecast = 'http://api.apixu.com/v1/forecast.json'
            params = {
                'key': '9838870ce6324daf95d161656180811',
                'q': location,
                'days': 7, # Forecast range
                'lang': 'es'
        }

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

    def _get_weather(self, city_name, forecast, date, info_required):
        """
        Get weather class.

        @param city: City to get weather.
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        result, result_info_dic = -1, {}

        # Checks if it is in local
        '''
        if True:
            return 0, result_info_dic
        '''

        # Make URL requests
        for source in self._SOURCE_LIST:
            result, result_info_dic = self._URL_request(source, city_name)
            if (result == 0): # URL request success
                #result_info_dic = self._format_change(source, result_info_dic) # Change weather format
                self._save_json(result_info_dic) # Saves the content in a local file
                break # Stops the URL requests

        return result, result_info_dic

    def _manage_weather(self, goal_vec):
        """
        Manager of the weather class. It updates the result and result_info.
        Examples:
        madrid/forecast/tomorrow/basic/101
        sydney/current/0/is_day/011

        @param goal_vec: Vector of the goal.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        if(len(goal_vec) >= self._GOAL_MAX_SIZE-1): # Check if all min fields are completed

            ############## Check weather ################
            self._city_name = goal_vec[0] # City name
            self._forecast = goal_vec[1] # Forecast or current info
            self._date = goal_vec[2] # Date
            self._info_required = goal_vec[3] # Info wanted

            result, result_info_dic = self._get_weather(self._city_name, self._forecast, self._date, self._info_required) # Get the weather info
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
