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

from time_weather_skill.weather_apixu import Apixu

pkg_name = 'time_weather_skill'

class Weather():

    """
    Weather class.
    """

    def __init__(self):
        """
        Init method.
        """

        # Class variables
        self.__city = ""
        self.__result = -1 # Weather result
        self.__result_info = "" # Weather info

        # Weather sources
        self.__source_list = ['apixu', 'source1', 'source2'] # List of the sources
        self.__source = self.__source_list[0] # Actual source. First source by default
        self.__apixu = Apixu()
        #self.__source1 = source1()...

    def _update_source(self, source):
        """
        Update the weather source variable.

        If the source exists, it updates. If not, the variable gets empty.

        @param source: New source.
        """

        # Search if the source is in the list
        for s in self.__source_list:
            if(source == s):
                self.__source = source # If the source exists, it updates
                print('New source: ' + self.__source)
                break
        else:
            self.__source = '' # If the source does not exists, the variable gets empty
            print('NO source called ' + source)

    def _check_weather(self, city, date, info_required):
    	"""
        Checks the weather.

        It uses the source specified in self.__source. Sources can be extendable.

        @param city: City to calculate weather. It updates self.__city.
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.
        """

        # Update city
        self.__city = city

        # Check weather
        if (self.__source == 'apixu'): # Apixu
            print('Chosen apixu')
            #### Make staff ####
            self.__apixu._request(city)
            self.__apixu._get_info(date, info_required)

            # Update results
            self.__result = self.__apixu._return_result()
            self.__result_info = self.__apixu._return_info()
            ####################

        elif (self.__source == 'source1'): # Source1
            print('Chosen source1')
            #### Make staff ####

            # Update result 
            self.__result = -1
            self.__result_info = {'state': 'Source1 not found'}
            ####################

        else: # No source
            print("Weather source not exists, or bad written")
            # Update result
            self.__result = -1
            self.__result_info = {'state': 'Weather source not exists, or bad written'}

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

    def _return_source(self):
        """
        Returns the actual weather source.
        """

        return self.__source

if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")

        weather = Weather() # Create object Weather
        weather._update_source('apixu') # Choose a weather source
        weather._check_weather('Madrid', 'today', 'basic') # Check weather in the specified city

    except rospy.ROSInterruptException:
        pass
