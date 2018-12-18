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
from time_weather_skill.timeClass import Time # Time class
from time_weather_skill.weatherClass import Weather # Weather class
from time_weather_skill.general_functions import makeCA_info, makeCA_gesture_info # Voice communication CA functions
from time_weather_skill.create_json import CreateJson # For creating and reading JSON files

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

city_name_def = 'Madrid' # City by default

class TimeWeatherSkill(Skill):
    """
    TimeWeather skill class.
    """

    # Feedback and result of this skill
    _feedback = time_weather_skill.msg.TimeWeatherFeedback()
    _result = time_weather_skill.msg.TimeWeatherResult()

    _goal_max_size = 5 # Max size of the goal

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
        self._date = "" # "today"
        self._info_required = "" # "basic"
        self._display = "" # "000"

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

    def manage_time(self, goal_vec):
        """
        Manager of the time class. It updates the result and result_info.

        @param goal_vec: Vector of the goal.
        """

        print("----- Chosen time -----")

        if(len(goal_vec)>=2): # If specified, it takes the city, if not, it uses the last city used
            self._city_name = goal_vec[1] # Register time
        else:
            self._city_name = ''

        self._time_var._check_time(self._city_name) # Check time

        self._result.result = self._time_var._return_result() # Get result
        self._result_info_dic = self._time_var._return_info() # Result_info = time info

    def manage_display(self, display):
        """
        Manager of the display. It updates the result and result_info.

        @param display: String of the display goal.
        """

        ############# Manage displays ###############
        display_vec = list(display) # Divides goal by fields
        self._screen = int(display_vec[0])
        self._movement = int(display_vec[1])
        self._voice = int(display_vec[2])
        #############################################

        ################# Screen ####################
        if(self._screen == 1):
            conditions_data = CreateJson() # Create object to read JSON
            conditions_dic = conditions_data.load('english_conditions_data') # Load JSON

            if(conditions_dic != -1): # JSON file found
                code, icon, is_day, date, city_name, avg_temp_c = '', '', 1, '', '', '' # Initialize text variables
                if 'code' in self._result_info_dic:
                    code = self._result_info_dic['code'] # Get condition code
                for n in conditions_dic: # Search the name of the icon, using the code
                    if(n['code'] == code):
                        icon = n['icon']
                        break
                if 'is_day' in self._result_info_dic:
                    is_day = self._result_info_dic['is_day'] # Get day condition (for finding weather icon)
                if 'date' in self._result_info_dic:
                    date = self._result_info_dic['date'] # Get date
                if 'city_name' in self._result_info_dic:
                    city_name = self._result_info_dic['city_name'] # Get city name
                if 'avg_temp_c' in self._result_info_dic:
                    avg_temp_c = self._result_info_dic['avg_temp_c'] # Get average temperature
                #print(code, icon, is_day, date, city_name, avg_temp_c)

            else:
                print('conditions_data JSON not found')
        #############################################

        ################# Movement ##################
        if(self._movement == 1):
            print('makeCA_gesture_info')
            ca_gesture_info = makeCA_gesture_info('alz_talking_03')
            self.ca_pub.publish(ca_gesture_info)
            rospy.sleep(1)
        #############################################

        ################## Voice ####################
        if(self._voice == 1):
            print('voice')
            weather_speech, date_speech, city_name_speech, text_speech, temp_speech, precip_speech = "", "", "", "", "", ""
            for key in self._result_info_dic:
                if(key == 'date'):
                    date_vec = self._result_info_dic[key].split("-") # Divides date by year-month-day
                    date_speech = "El dia " + str(date_vec[2]) + " del " + str(date_vec[1])
               
                if(key == 'city_name'):
                    city_name_speech = "en " + str(self._result_info_dic[key]) + " "
                
                if(key == 'text'):
                    text_speech = "estara " + str(self._result_info_dic[key]) + " "
                
                if(key == 'avg_temp_c'):
                    temp_vec = str(self._result_info_dic[key]).split(".") # Divides temp: 11.4 -> 11 and 4
                    temp_speech = "hara una temperatura de " + str(temp_vec[0]) + " coma " + str(temp_vec[1]) + " grados "

                weather_speech = date_speech + city_name_speech + text_speech + temp_speech + precip_speech
            ca_info = makeCA_info(weather_speech)
            self.ca_pub.publish(ca_info)
            rospy.sleep(1)
        #############################################

    def manage_weather(self, goal_vec):
        """
        Manager of the weather class. It updates the result and result_info.
        Examples:
        weather/Madrid/today/basic/101
        time/Madrid

        @param goal_vec: Vector of the goal.
        """

        print("----- Chosen weather -----")
        if(len(goal_vec) >= _goal_max_size-1): # Check if all fields are completed

            ############## Check weather ################
            self._city_name = goal_vec[1] # City name
            self._date = goal_vec[2] # Date
            self._forecast = goal_vec[3] # Forecast or current info
            self._info_required = goal_vec[4] # Info wanted

            self._weather_var._update_source('apixu') # Choose a weather source
            self._weather_var._check_weather(self._city_name, self._date, self._forecast, self._info_required) # Check weather in the specified city

            # Empty the dictionary
            self._result_info_dic = {}
            # Fill result_dic and result
            self._result.result = self._weather_var._return_result() # Get result
            self._result_info_dic = self._weather_var._return_info() # Result_info = weather info
            #############################################
            if(self._result.result == -1):
                return False
            else:
                return True

        else:
            print("Goal size not completed")
            self._result.result = -1 # Fail
            self._result_info_dic = {}
            return False

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received

        @param goal: time_weather_skill goal.
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
                if (goal_vec[0] == "time"): # Time goal
                    #################### Time ######################
                    self.manage_time(goal_vec)
                    ################################################

                elif (goal_vec[0] == "weather"): # Weather goal
                    ################### Weather ####################
                    weather = self.manage_weather(goal_vec)
                    if(weather == False): # Check if weather has been requested
                        print("==== Weather ERROR ====")
                        self._result.result = -1 # Fail
                    else:
                        ################ Display ###################
                        if(len(goal_vec) >= self._goal_max_size): # Check if the display field is completed
                            self.manage_display(goal_vec[self._goal_max_size-1])
                        else:
                            print("Display not specified")
                            self._result.result = -1 # Fail
                        ############################################
                        
                    ################################################

                else: # Wrong goal
                    print("Goal not correct")
                    self._result.result = -1 # Fail
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
        for key in self._result_info_dic:
            kvp = KeyValuePair()
            kvp.key = str(key)
            kvp.value = str(self._result_info_dic[key])
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
        rospy.init_node(skill_name)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = TimeWeatherSkill()
        rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
