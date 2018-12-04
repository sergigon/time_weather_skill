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

from skill.skill import Skill, NATURAL
from std_msgs.msg import String
from time_weather_skill.timeClass import Time

skill_name = "time_weather_skill"
city_name = 'Madrid'

class TimeWeatherSkill(Skill):

    """
    PeopleMonitor skill class.
    """

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._time_var = Time()

        # init the skill
        Skill.__init__(self, skill_name, NATURAL)
                              
    
    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """

        # publishers and subscribers
        self._time_var.create_msg_srv() # Create the time publisher (latch style)


    def run(self):
        """
        Spinner of the node.
        """

        while not rospy.is_shutdown():
            if self._status == self.RUNNING:
                ############ Publish time #############
                self._time_var._check_time(city_name)
                self._time_var._publish_time_state()
                #######################################

                # sleep some seconds
                rospy.sleep(5)


if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name)

        # create and spin the node
        node = TimeWeatherSkill()
        node.run()
    except rospy.ROSInterruptException:
        pass
