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

from skill.skill import Skill, ActionlibException, NATURAL
from time_weather_skill.timeClass import Time
from time_weather_skill.weatherClass import Weather

import rospy
import roslib
import importlib
import actionlib
#import pdb; pdb.set_trace()

# Messages
from std_msgs.msg import String, Empty
import multimedia_msgs.msg
import time_weather_skill.msg
from common_msgs.msg import KeyValuePair


# Skill variables
# Package name
pkg_name = 'time_weather_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "time_weather_skill"

city_name_def = 'Madrid' # City by default

class TimeWeatherSkill(Skill):
    """
    TimeWeather skill class.
    """

    # Feedback and result of this skill
    #_feedback = multimedia_msgs.msg.MultimediaFeedback() # getattr(multimedia_msgs.msg, server_feedback_str) #my_import(server_module_str, server_feedback_str) # (multimedia_msgs.msg.{Action_name}Feedback)
    #_result = multimedia_msgs.msg.MultimediaResult() # getattr(multimedia_msgs.msg, server_result_str) # (multimedia_msgs.msg.{Action_name}Result)

    _feedback = time_weather_skill.msg.TimeWeatherFeedback() # getattr(multimedia_msgs.msg, server_feedback_str) #my_import(server_module_str, server_feedback_str) # (multimedia_msgs.msg.{Action_name}Feedback)
    _result = time_weather_skill.msg.TimeWeatherResult() # getattr(multimedia_msgs.msg, server_result_str) # (multimedia_msgs.msg.{Action_name}Result)


    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir
        self._result_info_dic = {} # Dictionary container

        ## common variables
        self._city_name = city_name_def # City to find the weather
        ## time variables
        self._time_var = Time() # Time object
        ## weather variables
        self._weather_var = Weather() # Weather object
        self._display = "000"
        self._date = "today"
        self._info_required = "basic"

        # init the skill
        Skill.__init__(self, skill_name, NATURAL)


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        print("create_msg_srv() called")

        # publishers and subscribers

        # servers and clients
        # Si el servidor actionlib no se ha inicializado:

        if not self._as:
            self._as = actionlib.SimpleActionServer(skill_name, time_weather_skill.msg.TimeWeatherAction, execute_cb=self.execute_cb, auto_start=False)
            # start the action server
            self._as.start()

    def shutdown_msg_srv(self):
        """
        This function has to be implemented in the children.
        """

        # publishers and subscribers
        # FIXME: do not unregister publishers because a bug in ROS
        # self.__test_pub.unregister()

        # servers and clients
            
        print("shutdown_msg_srv() called")

    def manage_time(self, goal_vec):
        """
        Manager of the time class. It updates the result and result_info

        @param goal_vec: Vector of the goal.
        """

        print("Chosen time")

        if(len(goal_vec)>=2): # If specified, it takes the city, if not, it uses the last city used
            self._city_name = goal_vec[1] # Register time
        else:
            self._city_name = ''

        self._time_var._check_time(self._city_name) # Check time

        self._result.result = self._time_var._return_result() # Get result
        self._result_info_dic = self._time_var._return_info() # Result_info = time state

    def manage_weather(self, goal_vec):
        """
        Manager of the weather class. It updates the result and result_info.
        Examples:
        weather/Madrid/today/basic/101
        time/Madrid

        @param goal_vec: Vector of the goal.
        """

        print("Chosen weather")
        if(len(goal_vec)>=5): # Check if all fields are completed

            ############## Check weather ################
            self._city_name = goal_vec[1] # City name
            self._date = goal_vec[2] # Date
            self._info_required = goal_vec[3] # Info wanted

            self._weather_var._update_source('apixu') # Choose a weather source
            self._weather_var._check_weather(self._city_name, self._date, self._info_required) # Check weather in the specified city

            # Empty the dictionary
            self._result_info_dic = {}
            # Fill result_dic and result
            self._result.result = self._weather_var._return_result() # Get result
            self._result_info_dic = self._weather_var._return_info() # Result_info = weather info
            #############################################

            ############# Manage displays ###############
            self._display = goal_vec[4] # Display

            #############################################
            #    self._result.result = 0 # Success

        else:
            print("Goal size not completed")
            self._result.result = -1 # Fail
            self._result_info_dic = {'state': 'error: Goal size not completed'}

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received
        """

        # default values (In progress)
        self._result.result = -1 # Error
        #self._result.result_info = None # Empty
        self._feedback.feedback = 0

        ############### Si la skill esta activa: ###################
        if self._status == self.RUNNING:
            print ("RUNNING...")

            try:
                ######### Si el goal esta en estado Preempted (es
                # decir, hay un goal en cola o se cancela el goal
                # actual), activo la excepcion #####################
                if self._as.is_preempt_requested():
                    print("Preempt requested")
                    raise ActionlibException
                #==================================================#
                
                ################# Proceso el goal ##################
                goal_vec = goal.command.split("/") # Divides goal by fields

                # Checks goal
                if (goal_vec[0] == "time"): # Time
                    #################### Time ######################
                    self.manage_time(goal_vec)
                    ################################################

                elif (goal_vec[0] == "weather"): # Weather
                    ################### Weather ####################
                    self.manage_weather(goal_vec)
                    ################################################

                else: # Bad goal
                    print("Goal not correct")
                    self._result.result = -1 # Fail
                #==================================================#
            
            ######### Si se ha hecho un preempted o cancel: ########
            except ActionlibException:
                rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
                # FAIL
                self._result.result = 1
            #======================================================#
            
        #==========================================================#
        ############# Si la skill no esta activa: ##################
        else:
            print("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            # ERROR
            self._result.result = -1
        #==========================================================#
        
        # Envio el feedback al final del loop
        self._as.publish_feedback(self._feedback)
        
        #### Envio del resultado y actualizacion del status del goal ###
        # Use the result_info dictionary for filling the result_info variable
        kvp_list = [] # Auxiliar list for the KeyValuePair values
        for key in self._result_info_dic:
            kvp = KeyValuePair()
            kvp.key = str(key)
            kvp.value = str(self._result_info_dic[key])
            kvp_list.append(kvp)
            print(kvp_list)

        # Empty the result_info
        size = len(self._result.result_info)
        for x in range(0, size):
            self._result.result_info.pop()
        # Fill the result_info with the dictionary
        self._result.result_info = kvp_list
        
        # Send result
        if self._result.result == 0:
            rospy.logdebug("setting goal to succeeded")
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._as.set_preempted(self._result)
        print("### Result sent ###")
        #==============================================================#


        # Inicializacion variables



if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = TimeWeatherSkill()
        #node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
