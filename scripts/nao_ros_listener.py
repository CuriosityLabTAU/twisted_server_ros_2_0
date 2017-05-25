import json
import rospy
from std_msgs.msg import String
from nao_alproxy import NaoALProxy


class NaoListenerNode():
    def __init__(self):
        self.nao_alproxy = NaoALProxy()
        self.nao_alproxy.start_nao()
        self.publisher = rospy.Publisher('nao_state', String, queue_size=10)
        rospy.init_node('nao_listener_node') #init a listener:
        rospy.Subscriber('nao_commands', String, self.callback_nao_command)
        #rospy.Subscriber('nao_state', String, self.callback_nao_state)
        rospy.spin() #spin() simply keeps python from exiting until this node is stopped

    def callback_nao_command (self, data):
        print("callback_robotator", data.data)
        message = data.data
        rospy.loginfo(message)
        self.nao_alproxy.parse_message(message)
        print("finished", message)
        self.publisher.publish(data.data) #publish to nao_state to indicate that the robot command is complete

    def callback_nao_state (self, data):
        print("noa_ros_listener callback_nao_state", data.data)

if __name__ == '__main__':
    try:
        nao = NaoListenerNode()
    except rospy.ROSInterruptException:
        pass