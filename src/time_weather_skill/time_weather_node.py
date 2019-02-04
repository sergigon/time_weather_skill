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
from time_weather_skill.general_functions import makeCA_etts_info, makeCA_gesture_info # Voice communication CA functions
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

# Exceptions
class GoalError(Exception):
    pass
class GeneralError(Exception):
    pass

class TimeWeatherSkill(Skill):
    """
    TimeWeather skill class.
    """

    # Feedback and result of this skill
    _feedback = time_weather_skill.msg.TimeWeatherFeedback()
    _result = time_weather_skill.msg.TimeWeatherResult()

    # Constants
    _GOAL_MAX_SIZE = 4 # Max size of the goal
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

        # Local paths
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path(pkg_name) + '/' # Package path
        self._data_path = self._pkg_path + 'data/' # Data path
        #self._icons_path = self._data_path + 'weather_icons/' # Icons path
        self._conditions_path = self._data_path + 'conditions/' # Conditions path
        # Tablet paths
        self._icons_path = 'image/weather/' # Icons path

        # Icons settings
        self.weather_icon_set = 'wikipedia'
        self.weather_icon_size = '480x480'

        # init the skill
        Skill.__init__(self, skill_name, NATURAL)

    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        print("create_msg_srv() called")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher(
            "hri_manager/ca_activations", CA, queue_size=10)# CA publisher

        self.ca_deactivation_pub = rospy.Publisher(
            "hri_manager/ca_deactivations", String, queue_size=10) # CA deactivation publisher

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

    def _manage_display(self, forecast_type, date_i, display, standard_weather_dic):
        """
        Manager of the display.
    
        @param display: String of the display goal.
        @param standard_weather_dic: Input weather dicionary in standard format.
        """
        print('###### Managing display ######')

        ############# Manage displays ###############
        display_vec = list(display) # Divides goal by fields
        screen = int(display_vec[0])
        movement = int(display_vec[1])
        voice = int(display_vec[2])
        #############################################

        # Initialize fields content and checks if all is well readed
        try:
            country_code = standard_weather_dic['common']['country_code'] #
            city_name = standard_weather_dic['common']['city_name'] #
            source = standard_weather_dic['common']['source'] #
            if(forecast_type == 'current'):
                date = standard_weather_dic['current']['date'] #
                day = str(DatetimeManager(date).day_int())
                month = str(DatetimeManager(date).month_int())
                year = str(DatetimeManager(date).year_int())
                temp_c = standard_weather_dic['current']['temp_c'] #
                int_temp_c, dec_temp_c = str(temp_c).split('.')
                precip_mm = standard_weather_dic['current']['precip_mm'] #
                is_day = standard_weather_dic['current']['is_day'] #
                text = standard_weather_dic['current']['text'] #
                code = standard_weather_dic['current']['code'] #
                icon = standard_weather_dic['current']['icon'] #
                
            if(forecast_type == 'forecast'):
                date = standard_weather_dic['forecast']['forecastday'][date_i]['date'] #
                day = str(DatetimeManager(date).day_int())
                month = str(DatetimeManager(date).month_int())
                year = str(DatetimeManager(date).year_int())
                avgtemp_c = standard_weather_dic['forecast']['forecastday'][date_i]['avgtemp_c'] #
                int_avgtemp_c, dec_avgtemp_c = str(avgtemp_c).split('.')
                mintemp_c = standard_weather_dic['forecast']['forecastday'][date_i]['mintemp_c'] #
                int_mintemp_c, dec_mintemp_c = str(mintemp_c).split('.')
                maxtemp_c = standard_weather_dic['forecast']['forecastday'][date_i]['maxtemp_c'] #
                int_maxtemp_c, dec_maxtemp_c = str(maxtemp_c).split('.')
                totalprecip_mm = standard_weather_dic['forecast']['forecastday'][date_i]['totalprecip_mm'] #
                is_day = 1
                text = standard_weather_dic['forecast']['forecastday'][date_i]['text'] #
                code = standard_weather_dic['forecast']['forecastday'][date_i]['code'] #
                icon = standard_weather_dic['forecast']['forecastday'][date_i]['icon'] #

        except KeyError as e:
            rospy.logwarn('[weatherClass] Manage display ERROR: Key Error (%s)' % e)
            return -1
        except IndexError as e:
            rospy.logwarn('[weatherClass] Manage display ERROR: Index Error (%s)' % e)
            return -1

        ################# Screen ####################
        if(screen == 1):
            ##################################################
            # Info to show:                                  #
            # + Current:                                     #
            #     - country_code (show country name)         #
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
            #     - country_code (show country name)         #
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

            # Initialize screen variables
            image_url, image_type = '', '' 
            try:
                # Icon search
                if(self._ICON_OBTENTION == 1): # Local tablet search
                    rospy.loginfo('Using local icons')
                    # Create filepaths
                    cond_source_filepath = self._conditions_path + self._CONDITIONS_FILENAME_STR + source + '.csv'
                    cond_std_filepath = self._conditions_path + self._CONDITIONS_FILENAME_STR + 'standard' + '.csv'
                    # Search icon
                    std_code = csv_reader_IO(cond_source_filepath, 'source code', code, 'standard code') # Search standard code
                    icon_file = csv_reader_IO(cond_std_filepath, 'standard code', std_code, self.weather_icon_set + ' icon') # Search standard icon file
                    # Create icon path
                    if (icon_file != -1):
                        image_url = self._icons_path + '%s/%s/%s/%s' % (self.weather_icon_set, self.weather_icon_size, ('day' if (is_day == 1) else 'night'), icon_file)
                        image_type = 'image'
                    else:
                        raise GeneralError('There has been an error searching the icon')
                else: # Internet search
                    rospy.loginfo('Using internet icons')
                    image_url, image_type = icon, 'web'

                # Show tablet info
                if(forecast_type == 'current'):
                	print(country_code, city_name, date, temp_c, image_url)
                	ca_info = makeCA_tablet_info(image_url, image_type)
                elif(forecast_type == 'forecast'):
                	print(country_code, city_name, date, avgtemp_c, mintemp_c, maxtemp_c, image_url)
                	ca_info = makeCA_tablet_info(image_url, image_type)
                
                self.ca_pub.publish(ca_info)
                rospy.sleep(1)
            except GeneralError as e: # General Error
                rospy.logwarn('[%s] ManageDisplay: Not displaying on tablet (%s)' % (skill_name, e))
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
                weather_speech = 'En ' + str(city_name) + ' hay ' + str(int_temp_c) + ' coma ' + str(dec_temp_c) + ' grados'
            if(forecast_type == 'forecast'):
                date_speech = ''
                if(date_i == 0):
                    date_speech = 'Hoy'
                elif(date_i == 1):
                    date_speech = 'Mañana'
                else:
                    date_speech = 'El día ' + str(day)
                weather_speech = (str(date_speech) + ' van a haber temperaturas máximas de ' + str(int_maxtemp_c) + ' coma '
                    + str(dec_maxtemp_c) + ' grados y mínimas de ' + str(int_mintemp_c) + ' coma '
                    + str(dec_mintemp_c) + ' en ' + str(city_name))

            print('Etts output:')
            print(' -> ' + weather_speech + ' <-')

            ca_info = makeCA_etts_info(str(weather_speech))
            self.ca_pub.publish(ca_info)
            rospy.sleep(1)
        #############################################

    def _goal_handler(self, goal):
        """
        Function to handle the goal. If some field is wrong the result is -1, else 0.
        """
        location, forecast_type, date, display = '', '', '', ''

        try:
            goal_vec = goal.command.split("/") # Divides goal by fields
            location = goal_vec[0] # Location ('madrid', or 'madrid, es')
            forecast_type = goal_vec[1] # Forecast or current info
            date = goal_vec[2] # Date
            display = goal_vec[3] # Display info
            # Check location
            # Check forecast_type
            if(forecast_type != 'current' and forecast_type != 'forecast'):
                raise GoalError('%s is not a forecast type' % forecast_type)
            # Check date
            try:
                date = date_text2num(date) # Date
            except ValueError as e:
                raise GoalError('%s is not a number' % date)
            # Check display
            display_vec = list(display) # Divides goal by fields
            if(len(display_vec)!=3):
                raise GoalError('Display has not the appropriate size: %s' % display)
            for display_i in display_vec:
                if(display_i != '0' and display_i != '1'):
                    raise GoalError('Display has not the appropriate value: %s' % display)

        except IndexError as e:
            rospy.logerr('[%s] Goal Handler: Goal NOT completed (%s)' % (skill_name, e))
            return -1, location, forecast_type, date, display

        except GoalError as e:
            rospy.logerr('[%s] Goal Handler: Goal content wrong (%s)' % (skill_name, e))
            return -1, location, forecast_type, date, display

        # Goal appropriate
        return 0, location, forecast_type, date, display

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
                result, location, forecast_type, date, display = self._goal_handler(goal)

                if(result == 0):
                    ################ Weather ################
                    # Get weather standard data
                    self._result.result, result_info_dic = Weather(self._data_path)._request_weather(location, forecast_type, date) # Get the weather info
                    if(self._result.result == 0): # Success
                        ################ Display ################
                        self._manage_display(forecast_type, date, display, result_info_dic)
                        
                    else: # Fail
                        rospy.logerr('[%s] Weather ERROR ' % pkg_name)

                else:
                    rospy.logerr('[%s] Goal not appropriate ' % pkg_name)
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
