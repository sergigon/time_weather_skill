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
from time_weather_skill.weatherClass import Weather # Weather class
from time_weather_skill.general_functions import makeCA_info, makeCA_gesture_info # Voice communication CA functions
from time_weather_skill.sys_operations import SysOperations
from time_weather_skill.datetime_manager import DatetimeManager
from time_weather_skill.general_functions import *
from time_weather_skill.csv_reader import *

import rospkg
import rospy
import roslib
import importlib
import actionlib
#import pdb; pdb.set_trace()

# Messages
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
import time_weather_skill.msg
from common_msgs.msg import KeyValuePair

# Skill variables
# Package name
pkg_name = 'time_weather_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "time_weather_skill"

class TimeWeatherSkill(Skill):
    """
    TimeWeather skill class.
    """

    # Feedback and result of this skill
    _feedback = time_weather_skill.msg.TimeWeatherFeedback()
    _result = time_weather_skill.msg.TimeWeatherResult()

    # Constants
    _GOAL_MAX_SIZE = 5 # Max size of the goal
    _ICON_OBTENTION = 1 # Determines the obetention of icons (0: Internet, 1: Local)

    # Filenames
    _CONDITIONS_FILENAME_STR = 'conditions_codes_' # Part name of the file to read the conditions info

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir

        # Paths
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path(pkg_name) + '/' # Package path
        self._data_path = self._pkg_path + 'data/' # Data path
        self._icons_path = self._data_path + 'weather_icons/' # Icons path
        self._conditions_path = self._data_path + 'conditions/' # Conditions path

        # Icons settings
        self.weather_icon_set = 'wikipedia'
        self.weather_icon_size = '640x640'

        # init the skill
        Skill.__init__(self, skill_name, NATURAL)


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        print("create_msg_srv() called")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher("hri_manager/ca_activations", CA, queue_size=10) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher("hri_manager/ca_deactivations", String, queue_size=10) # CA deactivation publisher

        #self.sub_response = rospy.Subscriber(robot + "hri_manager/response", CA, self.response_callback) # CA subscriber

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

    def _manage_display(self, forecast_type, date_i, display, weather_dic):
        """
        Manager of the display.
    
        @param display: String of the display goal.
        @param weather_dic: Input weather dicionary.
        """
        print('###### Managing display ######')

        ############# Manage displays ###############
        display_vec = list(display) # Divides goal by fields
        screen = int(display_vec[0])
        movement = int(display_vec[1])
        voice = int(display_vec[2])
        #############################################

        # Initialize fields content
        # Common
        country_name, city_name, date, day, month, year, temp_c, avgtemp_c = '', '', '', '', '', '', '', ''
        # Screen
        code, icon, is_day = '', '', 1 # By default it is day (for images)
        # Voice
        text = ''
        # Params
        date_i = date_text2num(date_i)

        # Fill screen fields content
        if 'country_name' in weather_dic:
            country_name = weather_dic['country_name'] #
        if 'city_name' in weather_dic:
            city_name = weather_dic['city_name'] #
        if 'date' in weather_dic:
            date = weather_dic['date'] #
            day = DatetimeManager(date).day()
            month = DatetimeManager(date).month()
            year = DatetimeManager(date).year()
        if 'temp_c' in weather_dic:
            temp_c = weather_dic['temp_c'] #
        if 'avgtemp_c' in weather_dic:
            avgtemp_c = weather_dic['avgtemp_c'] #
        if 'is_day' in weather_dic:
            is_day = weather_dic['is_day'] #
        if 'text' in weather_dic:
            text = weather_dic['text'] # 

        ################# Screen ####################
        if(screen == 1):
            ##################################################
            # Info to show:                                  #
            # + Current:                                     #
            #     - country_name (show country name)         #
            #     - city_name (show city name)               #
            #     - date (show date)                         #
            #     - day (show day)                           #
            #     - month (show day)                         #
            #     - year (show day)                          #
            #     - temp_c (show temperature in celsius)     #
            #     - code (show photo)                        #
            #     - source (needed to make code conversion)  #
            #     - is_day (change photo if night)           #
            # + Forecast:                                    #
            #     - country_name (show country name)         #
            #     - city_name (show city name)               #
            #     - date (show date)                         #
            #     - day (show day)                           #
            #     - month (show day)                         #
            #     - year (show day)                          #
            #     - avgtemp_c (show temperature in celsius)  #
            #     - code (show photo)                        #
            #     - source (needed to make code conversion)  #
            ##################################################
            print('>> Screen selected')

            # Searchs the icon image in local
            if(self._ICON_OBTENTION == 1):
                rospy.loginfo('Using local icons')
                if ('code' in weather_dic and 'source' in weather_dic):
                    code, source = weather_dic['code'], weather_dic['source'] #
                    # Create filepaths
                    cond_source_filepath = self._conditions_path + self._CONDITIONS_FILENAME_STR + source + '.csv'
                    cond_std_filepath = self._conditions_path + self._CONDITIONS_FILENAME_STR + 'standard' + '.csv'
                    # Search icon
                    std_code = csv_reader_IO(cond_source_filepath, 'source code', code, 'standard code') # Search standard code
                    icon_file = csv_reader_IO(cond_std_filepath, 'standard code', std_code, self.weather_icon_set + ' icon') # Search standard icon file
                    # Create icon path
                    if (icon_file != -1):
                        icon = self._icons_path + '%s/%s/%s/%s' % (self.weather_icon_set, self.weather_icon_size, ('day' if (is_day == 1) else 'night'), icon_file)
                    else:
                        rospy.logerr('There has been an error searching the icon')
                else:
                    rospy.logerr('To activate local icon, please ask for code and source info')
            else:
                rospy.loginfo('Using internet icons')
                if 'icon' in weather_dic:
                    icon = weather_dic['icon'] #
                else:
                    rospy.logerr('To activate internet icon, please ask for icon info')

            print(country_name, city_name, date, temp_c, avgtemp_c, code, icon)
        #############################################

        ################# Movement ##################
        if(movement == 1):
            print('>> Movement selected')
            ca_gesture_info = makeCA_gesture_info('alz_talking_03')
            self.ca_pub.publish(ca_gesture_info)
            rospy.sleep(1)
        #############################################

        ################## Voice ####################
        if(voice == 1):
            ##################################################
            # Info to show:                                  #
            # + Current:                                     #
            #     - city_name (show city name)               #
            #     - date (show date)                         #
            #     - temp_c (show temperature in celsius)     #
            #     - text (show condition)                    #
            # + Forecast:                                    #
            #     - city_name (show city name)               #
            #     - date (show date)                         #
            #     - avgtemp_c (show temperature in celsius)  #
            #     - text (show condition)                    #
            ##################################################
            print('>> Voice selected')

            if(forecast_type == 'current'):
            	weather_speech = 'En ' + str(city_name) + ' hay ' + str(temp_c) + ' grados con ' + str(text)
            if(forecast_type == 'forecast'):
            	date_speech = ''
            	if(date_i == 0):
                    date_speech = 'Hoy'
                elif(date_i == 1):
                    date_speech = 'Mañana'
                else:
                    date_speech = 'El día ' + str(day)
            	weather_speech = date_speech + ' va a haber ' + str(temp_c) + ' grados con ' + str(text) + ' en ' + str(city_name)

            print('Etts output:')
            print(' -> ' + weather_speech + ' <-')

            ca_info = makeCA_info(weather_speech)
            self.ca_pub.publish(ca_info)
            rospy.sleep(1)
        #############################################

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received

        @param goal: time_weather_skill goal.
        """

        # default values (In progress)
        self._result.result = -1 # Error
        self._result.result_info = [] # Empty
        result_info_dic = {} # Dictionary to fill result_info
        self._feedback.feedback = 0

        ############### Si la skill esta activa: ###################
        if self._status == self.RUNNING:
            print ("RUNNING...")

            try:
                ############# State Preempted checking #############
                # Si el goal esta en estado Preempted (es decir,   #
                # hay un goal en cola o se cancela el goal         #
                # actual), activo la excepcion                     #
                ####################################################
                if self._as.is_preempt_requested():
                    print("Preempt requested")
                    raise ActionlibException
                #==================================================#
                
                ################ Processes the goal ################
                goal_vec = goal.command.split("/") # Divides goal by fields
                if(len(goal_vec) >= self._GOAL_MAX_SIZE-1): # Check if all fields (except display) are completed
                    ################### Weather ####################
                    location = goal_vec[0] # City name
                    forecast_type = goal_vec[1] # Forecast or current info
                    date = goal_vec[2] # Date
                    info_required = goal_vec[3] # Info wanted
                    # Get weather info
                    self._result.result, result_info_dic = Weather(self._data_path)._get_info(location, forecast_type, date, info_required)
                    if(self._result.result != 0): # Fail
                        rospy.logerr('[%s] Weather ERROR ' % pkg_name)
                    else:
                        if(len(goal_vec) >= self._GOAL_MAX_SIZE): # Check if all fields are completed
                            ################ Display ################
                            display = goal_vec[4] # Display info
                            self._manage_display(forecast_type, date, display, result_info_dic)
                        else:
                            rospy.logwarn('[%s] Display not specified ' % pkg_name)

                else:
                    rospy.logerr('[%s] Goal size not completed ' % pkg_name)
                    self._result.result = -1 # Fail
                    result_info_dic = {}

                #==================================================#
            
            ######### Si se ha hecho un preempted o cancel: ########
            except ActionlibException:
                rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
                # FAIL
                self._result.result = 1 # Preempted
            #======================================================#
            
        #==========================================================#
        ############# Si la skill no esta activa: ##################
        else:
            print("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            # ERROR
            self._result.result = -1 # Fail
        #==========================================================#
        
        # Envio el feedback al final del loop
        self._as.publish_feedback(self._feedback)
        
        #### Envio del resultado y actualizacion del status del goal ###
        # Use the result_info dictionary for filling the result_info variable
        kvp_list = [] # Auxiliar list for the KeyValuePair values
        for key in result_info_dic:
            kvp = KeyValuePair()
            kvp.key = str(key)
            kvp.value = str(result_info_dic[key])
            kvp_list.append(kvp)

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

        print("###################")
        print("### Result sent ###")
        print("###################")
        #==============================================================#


        # Inicializacion variables



if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name, log_level=rospy.DEBUG)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = TimeWeatherSkill()
        rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass