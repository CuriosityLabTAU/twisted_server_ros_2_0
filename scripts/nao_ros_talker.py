import json
import rospy
from std_msgs.msg import String
from nao_alproxy import NaoALProxy
import sys


class NaoTalkerNode():
    def __init__(self, nao_ip=None):
        self.nao_alproxy = NaoALProxy(nao_ip)
        self.publisher_nao_angles = rospy.Publisher('nao_angles_topic', String, queue_size=10)
        self.publisher_target_position =rospy.Publisher('nao_target_position', String, queue_size=10)
        rospy.init_node('nao_talker_node', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            print (hello_str)
            angles_str = str(self.nao_alproxy.get_angles())
            print(angles_str)
            target_position = str(self.nao_alproxy.get_target_position())
            rospy.loginfo(angles_str)
            self.publisher_nao_angles.publish(angles_str)
            self.publisher_target_position.publish(target_position)
            rate.sleep()


if __name__ == '__main__':
    try:
        # from command line: first input is an optional nao_ip
        if len(sys.argv) > 1:
            nao = NaoTalkerNode(sys.argv[1])
        else:
            nao = NaoTalkerNode()
    except rospy.ROSInterruptException:
        pass