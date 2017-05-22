import json
import rospy
from std_msgs.msg import String
from nao import Nao


class NaoListenerNode():
    def __init__(self):
        self.nao = Nao()
        self.nao.start_nao()
        self.publisher = rospy.Publisher('nao_state', String, queue_size=10)
        rospy.init_node('nao_listener_node') #init a listener:
        rospy.Subscriber('nao_commands_topic', String, self.callback_nao)
        rospy.spin() #spin() simply keeps python from exiting until this node is stopped

    def callback_nao (self, data):
        print("callback_robotator", data.data)
        message = data.data
        rospy.loginfo(message)
        self.nao.parse_message(message)

    #def publish_angeles(self):
    #    rospy.init_node('nao_angeles')

if __name__ == '__main__':
    try:
        nao = NaoListenerNode()
    except rospy.ROSInterruptException:
        pass