import json
import rospy
from std_msgs.msg import String
import time

from naoqi import ALProxy
import sys
import almath


class TwistedServer():

    def __init__(self):
        rospy.init_node('twisted_server')
        self.pub = rospy.Publisher('nao_commands_topic', String, queue_size=10)

    def send_message (self, message):
        rospy.loginfo(message)
        #pub = rospy.Publisher('nao_commands', String, queue_size=10)

        self.pub.publish(message)





