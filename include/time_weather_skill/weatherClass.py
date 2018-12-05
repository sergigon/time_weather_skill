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

from astral import Astral

from std_msgs.msg import String

pkg_name = 'time_weather_skill'
city_name_def = 'Madrid' # Ciudad por defecto

class Weather():

    """
    Time skill class.
    """

    def __init__(self, city_name=city_name_def): # Si no se especifica, se usa ciudad por defecto
        """
        Init method.

        @param city: City to calculate time. If not specified, it is used the city by default
        """

        # class variables
        self.__city_name = city_name_def
        self.__info = "" # Weather info

    def __update_city(self, city):
    	"""
    	Update the parameters with the new city.

    	@param city: New city.
    	"""
    	self.__city_name = city


    def _check_weather(self, city=""):
    	"""
        Checks if it is day or night.
        If not specified, it uses the last city used.

        Change the variable self.__state ('day' or 'night').

        @param city: City to calculate time. It updates self.__city.
        """

        # Update city if specified
        if (city!=""): # If empty, means not to update
            self.__update_city(city)
            print("city updated: " + self.__city_name)

        # Check weather


    def _get_info(self):
        """
        Return state
        """
        return self.__info


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")
    except rospy.ROSInterruptException:
        pass
