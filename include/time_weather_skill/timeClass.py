#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio González Díaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio González Díaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio González Díaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import rospy
import datetime

from astral import Astral

pkg_name = 'time_weather_skill'

class TimeAstral():

    """
    Time class that uses Astral class info.
    """

    def __init__(self):
        """
        Init method.
        """


    def _update_city_params(self, city_name):
        """
        Update the city parameters.

        @param location: City to update. Format: 'Madrid' or 'Madrid, Spain'. ?????????????

        @return city: City object.
        @return timezone: Timezone object.
        """

        astral = Astral() # Astral object

        # Update city parameters
        try:
            city = astral[city_name]
            timezone = city.timezone
        except:
            rospy.logwarn("TimeAstral ERROR: City not available")
            city = -1 # Error
            timezone = -1

        return city, timezone
        

    def _is_day(self, city_name):
    	"""
        Checks if it is day or night.

        @param city_name: City to calculate if it is day or night. Format: 'Madrid'

        @return result: Final result.
        @return result_info: Info result.
        """

        # Initialize results
        result, result_info = -1, ''

        # Update city parameters
        city, timezone = self._update_city_params(city_name)

        # Check time
        if(city != -1):
            sun = city.sun(date=datetime.date.today(), local=False)
            if sun['dawn'].replace(tzinfo=None) < datetime.datetime.now() < sun['dusk'].replace(tzinfo=None):
                result_info = 1
            else:
                result_info = 0
            result = 0

        else:
            result, result_info = -1, ''

        return result, result_info

    def get_info(self, city_name, info_required):
        """
        Get the info specified.

        @param city_name: City to get info. Format: 'Madrid'
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info: Info result.
        """

        # Initialize results
        result, result_info = -1, ''

        # Info list
        if(info_required == 'is_day'):
            result, result_info = self._is_day(city_name)

        return result, result_info


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "]: __main__")

        time_var = TimeAstral()
        
        result, result_info = time_var.get_info('Madrid', 'is_day') # Check day

        print(result)
        print(result_info)
        

    except rospy.ROSInterruptException:
        pass
