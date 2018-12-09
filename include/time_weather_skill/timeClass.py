#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Marcos Maroto Gomez"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Marcos Maroto Gomez"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Marcos Maroto Gomez"
__email__ = "marmarot@ing.uc3m.es"
__status__ = "Development"

import math
import rospy
import datetime

from astral import Astral

from std_msgs.msg import String

pkg_name = 'time_weather_skill'
city_name_def = 'Madrid' # Ciudad por defecto

class Time():

    """
    Time class.
    """

    def __init__(self): # Si no se especifica, se usa ciudad por defecto
        """
        Init method.
        """

        # class variables    
        self.__state = {} # Info
        self.__result = -1 # Result
        self.__city_name = ''

        # Astral variables
        self.__astral = Astral() # Astral object
        self.__astral.solar_depression = 'civil'
        self.__city = None
        self.__timezone = None
        

    def __update_city_params(self, city):
    	"""
    	Update the parameters with the new city.

    	@param city: New city.
    	"""
    	self.__city_name = city
        
        try:
            self.__city = self.__astral[self.__city_name]
            self.__timezone = self.__city.timezone
            self.__result = 0 # Success
        except:
            print("City not available")
            self.__result = -1 # Errors


    def _check_time(self, city):
    	"""
        Checks if it is day or night.
        If not specified, it uses the last city used.

        Change the variable self.__state ('day' or 'night').

        @param city: City to calculate time. It updates self.__city.
        """

        # Reset
        self.__state = {}

        # Update city parameters
        self.__update_city_params(city)

        # Check time
        if(self.__result == 0): # If result is success, searchs for state
            sun = self.__city.sun(date=datetime.date.today(), local=False)

            if sun['dawn'].replace(tzinfo=None) < datetime.datetime.now() < sun['dusk'].replace(tzinfo=None):
                self.__state = {'state': 'day', 'city_name': self.__city_name}
            else:
                self.__state = {'state': 'night', 'city_name': self.__city_name}
        else: # Error
            self.__state = {'state': 'error: City not available', 'city_name': self.__city_name}

    def _return_info(self):
        """
        Return state
        """
        return self.__state
        
    def _return_result(self):
        """
        Return result
        """
        return self.__result

if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "]: __main__")

        time_var = Time()
        
        time_var._check_time('Paris') # Check time
        result = time_var._return_result() # Get result
        info = time_var._return_info() + "/" + 'Paris' # Result_info = time/city

        print(result)
        print(info)
        

    except rospy.ROSInterruptException:
        pass
