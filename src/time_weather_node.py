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

import rospy
import roslib
import importlib
import actionlib

# Messages
from std_msgs.msg import String, Empty
import multimedia_msgs.msg

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
    _feedback = multimedia_msgs.msg.TestFeedback() # getattr(multimedia_msgs.msg, server_feedback_str) #my_import(server_module_str, server_feedback_str) # (multimedia_msgs.msg.{Action_name}Feedback)
    _result = multimedia_msgs.msg.TestResult() # getattr(multimedia_msgs.msg, server_result_str) # (multimedia_msgs.msg.{Action_name}Result)

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir

        ## time variables
        self._city_name = 'Madrid' # City to find the weather
        self._time_var = Time() # Time class

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
            self._as = actionlib.SimpleActionServer(skill_name, multimedia_msgs.msg.TestAction, execute_cb=self.execute_cb, auto_start=False)
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

    def run(self):
        """
        Spinner of the node.
        """

        while not rospy.is_shutdown():
            if self._status == self.RUNNING:
                ############ Publish time #############
                self._time_var._check_time(self._city_name)
                self._time_var._publish_time_state()
                #######################################

                # sleep some seconds
                rospy.sleep(5)

    def execute_time(self, goal):
        """
        Callback of the time node.
        """


    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received
        """
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # default values (In progress)
        self._result.result = 0
        self._feedback.feedback = 0

        ############### Si la skill esta activa: ###################
        if self._status == self.RUNNING:
            print ("RUNNING...")
            try:
                # Si el goal esta en estado Preempted (es decir, hay
                # un goal en cola o se cancela el goal actual),
                # activo la excepcion ##############################
                if self._as.is_preempt_requested():
                    print("Preempt requested")
                    raise ActionlibException
                #==================================================#
                
                ################ Proceso el goal ###################
                goal_vec = goal.command.split("/")
                goal_vec_size = len(goal_vec)
                for z in goal_vec:
                    print("Command: " + z)
                
                self._result.result = 1
                #==================================================#
            
            ######### Si se ha hecho un preempted o cancel: ########
            except ActionlibException:
                rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
                # FAIL
                self._result.result = -1
                self._feedback.feedback = -1
            #======================================================#
            
        #==========================================================#
        ############# Si la skill no esta activa: ##################
        else:
            print("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            # ERROR
            self._result.result = -2
            self._feedback.feedback = 0
        #==========================================================#
        
        # Envio el feedback al final del loop
        self._feedback.feedback = 0
        self._as.publish_feedback(self._feedback)
        
        #### Envio del resultado y actualizacion del status del goal ###
        print("Envío resultado")
        if self._result.result:
            rospy.logdebug("setting goal to succeeded")
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._as.set_preempted(self._result)
        #==============================================================#


        # Inicializacion variables



if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node('hola')

        # create and spin the node
        node = TimeWeatherSkill()
        #node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
