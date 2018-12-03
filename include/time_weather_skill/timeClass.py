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

pkg_name = 'timeClass'
city_name_def = 'Madrid' # Ciudad por defecto

class Time():

    """
    Time skill class.
    """

    def __init__(self, city_name=city_name_def): # Si no se especifica, se usa ciudad por defecto
        """
        Init method.

        @param city: City to calculate time. If not specified, it is used the city by default
        """

        # class variables
        self.__previous_state = ''      
        self.__state = ''
        self.__astral = Astral()

        self.__city_name = city_name
        self.__city = self.__astral[self.__city_name]
        self.__timezone = self.__city.timezone

        self.__astral.solar_depression = 'civil'


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """

        # publishers and subscribers        
        self.__time_pub = rospy.Publisher('time', String, latch=True, queue_size=1)

    def __update_city(self, city):
    	"""
    	Update the parameters with the new city.

    	@param city: New city.
    	"""
    	self.__city_name = city
        self.__city = self.__astral[self.__city_name]
        self.__timezone = self.__city.timezone


    def _check_time(self, city=""):
    	"""
        Checks if it is day or night.
        If not specified, it uses the last city used.

        Change the variable self.__state ('day' or 'night').

        @param city: City to calculate time. It updates self.__city.
        """

        # Update city if specified
        if (city!=""): # If empty, means not to update
        	self.__update_city(city)

        # Check time
        sun = self.__city.sun(date=datetime.date.today(), local=True)

        if sun['dawn'].replace(tzinfo=None) < datetime.datetime.now() < sun['dusk'].replace(tzinfo=None):
            self.__state = 'day'
        else:
            self.__state = 'night'


    def _publish_time_state(self):
        """
        Publish if new state received.
        """

        if self.__previous_state != self.__state:
            # update the state
            rospy.logdebug('[%s]: %s' % (pkg_name, self.__state))
            print('[%s]: %s' % (pkg_name, self.__state)) 

            # publish the new state
            self.__time_pub.publish(String(self.__state))
            self.__previous_state = self.__state
            print(self.__city_name)


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")
    except rospy.ROSInterruptException:
        pass
