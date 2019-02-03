#!/usr/bin/env python
# -*- coding: utf-8 -*-

# File: general_functions.py

__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import time
import roslib
import rospy


from std_msgs.msg import String, Int16, Empty
from interaction_msgs.msg import CA
from common_msgs.msg import KeyValuePair as kvpa

######################################################################

def location_divider(location):
    """
    Divide the location in city_name and country_name.

    @param location: Location to get weather. Format: 'Madrid' or 'Madrid, Spain'.

    @return city_name: Name of the city.
    @return country_name: Name of the country. If not specified, it returns ''.
    """

    # Initialize variables
    city_name, country_name = '', ''

    # Checks if location has specified country
    n_commas = location.find(',')
    if(n_commas <= 0): # NOT specified country
        city_name = location
    else: # Specified country
        city_name, country_name = location.split(",")
        if(country_name[0] == ' '): # Removes space character if necessary
            country_name = country_name[1:]

    return city_name, country_name

def location_combiner(city_name, country_name):
    """
    Combines the city_name and country_name in the location.

    @param city_name: Name of the city.
    @param country_name: Name of the country.

    @return location: Location to get weather.
    """

    # Initialize variables
    location = city_name + ', ' + country_name

    return location

def date_text2num(text):
    """
    Transforms the text into the corresponding number and converts to int.

    @param text: string date to fix.

    @return date: int date fixed.
    """

    if(text == ''):
        text = '-1'
    elif(text == 'today'):
        text = '0'
    elif(text == 'tomorrow'):
        text = '1'

    date = int(text) # Converts to int

    return date

def makeCA_info(etts_text):
    now = rospy.get_rostime().nsecs

    msg = CA()
    msg.type = "robot_giving_info"
    msg.ca_name = str(now)
    msg.duration = 0
    msg.priority = 1
    msg.emitter = "time_weather_ca"
    
    kvp = kvpa()
    kvp.key = "etts_text"
    kvp.value = etts_text
    msg.values.append(kvp)
    rospy.logdebug("Sending CA_info")
    return msg
    
def makeCA_gesture_info(gesture):
    now = rospy.get_rostime().nsecs

    msg = CA()
    msg.type = "robot_giving_info"
    msg.ca_name = str(now)
    msg.duration = 0
    
    kvp = kvpa()
    kvp.key = "gesture"
    kvp.value = gesture
    msg.values.append(kvp)
    rospy.logdebug("Sending CA_info")
    return msg

def activateCA(timestamp,grammar,tag):
    msg = CA()
    msg.type = "user_asking_for_info"
    msg.ca_name = timestamp #str(time.time())
    msg.duration = 1
    
    kvp = kvpa()
    kvp.key = "answer_type"
    kvp.value = "ASR"
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "grammar"
    kvp.value = grammar
    msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "answer_id"
    kvp.value = tag
    msg.values.append(kvp)

    print "Sending CA_grammar"
    return msg
    
def activateTouchCA(timestamp,tag):
    msg = CA()
    msg.type = "user_asking_for_info"
    msg.ca_name = timestamp #str(time.time())
    msg.duration = 1
    
    kvp = kvpa()
    kvp.key = "answer_type"
    kvp.value = "touch"
    msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "answer_id"
    kvp.value = tag
    msg.values.append(kvp)

    print "Sending CA_touch"
    return msg

def deactivateCA(timestamp):
    msg=String()
    msg.data = timestamp
    print "Delete CA_grammar"
    return msg

def sendCA_question(text, language='es', duration=0, priority=0, answer_type='ASR', answer_id=None, answer_time=10, answer_attempts=2, grammar=None):
    msg = CA()
    msg.ca_name=str(time.time())
    msg.type = "robot_asking_for_info"
    msg.duration = duration
    msg.priority = priority

    kvp = kvpa()
    kvp.key = "etts_text"
    kvp.value = text
    msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "etts_language"
    kvp.value = language
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "answer_type"
    kvp.value = answer_type
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "answer_id"
    kvp.value = answer_id
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "answer_time"
    kvp.value = str(answer_time)
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "answer_attempts"
    kvp.value = str(answer_attempts)
    msg.values.append(kvp)
    
    kvp = kvpa()
    kvp.key = "grammar"
    kvp.value = grammar
    msg.values.append(kvp)

    return msg
