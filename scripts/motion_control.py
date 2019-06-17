#!/usr/bin/env python
#!/usr/bin/env python

import roslib; roslib.load_manifest("dynamixel_hr_ros")
import rospy
from std_msgs.msg import *
from dynamixel_hr_ros.msg import *
import logging
import time
import numpy as np
# from affdex_msgs.msg import AffdexFrameInfo
from geometry_msgs.msg import PoseStamped

logging.basicConfig(level=logging.DEBUG)


class MotionControl:

    def __init__(self):
        #self.motor_list = [1,2,5,7,6,8]
        #self.motor_list = [1,2,5,7]
        self.motor_list  = {'skeleton': [0, 1, 4, 5, 6, 7], 'head_pose': [2], 'lip': [3], 'full': [0, 1, 2, 3, 4, 5, 6, 7], 'full_idx': [1, 2, 3, 4, 5, 6, 7, 8]}
        self.robot_angle_range = [[0.0, 5.0], #[1.1, 3.9],
                                  [2.8, 1.2],
                                  [3.1, 3.3], [1.8, 2.0], #[1.8, 2.0],
                                  [4.1, 0.9], [1.3, 3],#                                   [4.1, 0.9], [1.3, 3],
                                  [1, 4.1], [2.5, 3.75]]
        self.sensor_angle_range = [[-np.pi, np.pi], [0, np.pi/2],
                                   [-0.2, 0.2], [0, 254],
                                   [-np.pi/2, np.pi/2], [np.pi/2, 0],
                                   [-np.pi/2, np.pi/2], [0, np.pi/2]]
        # self.robot_kinect_angles = [0, 1, 0, 0, 2, 3, 4, 5]
        self.robot_kinect_angles = [0, 1, 0, 0, 4, 5, 2, 3] # complete mirror
        self.skeleton_angles = []
        self.motor_speeds = [1, 1, 6, 6, 6, 6, 6, 6]#[1, 1, 4, 4, 4, 4, 4, 4]# #[1, 1, 1, 4, 1, 1, 1, 1] #[3, 3, 5, 3, 3, 3, 3, 3]

        self.robot_angles = np.zeros([8])
        for i, rar in enumerate(self.robot_angle_range):
            self.robot_angles[i] = np.mean(rar)
        self.robot_angles[1] = 2.8

        # print(self.robot_angles)


    def start(self):
        # init a listener to kinect angles
        rospy.init_node('motion_control')
        rospy.Subscriber("skeleton_angles", String, self.kinect_callback)
        rospy.Subscriber("head_pose", PoseStamped, self.head_pose_callback)
        rospy.Subscriber("lip_angles", String, self.lip_callback)
        enabler = rospy.Publisher("/dxl/enable", Bool, queue_size=10)
        self.commander = rospy.Publisher("/dxl/command_position", CommandPosition, queue_size=10)
        time.sleep(1)
        enabler.publish(True)
        time.sleep(1)
        rospy.spin()

    def head_pose_callback(self, data):
        head_angle = -data.pose.orientation.w

        self.robot_angles[self.motor_list['head_pose'][0]] = self.map_angles(self.sensor_angle_range[2],
                                                                          self.robot_angle_range[self.motor_list['head_pose'][0]],
                                                                          head_angle)
        self.move_motors()


    def lip_callback(self, data):
        # print ['lip angles : ', data]
        self.robot_angles[self.motor_list['lip'][0]] = self.map_angles(
            self.sensor_angle_range[self.motor_list['lip'][0]], self.robot_angle_range[self.motor_list['lip'][0]], float(data.data))
        self.move_motors()

    def kinect_callback(self, data):
        self.skeleton_angles = np.array([float(x) for x in data.data.split(',')])
        for motor in self.motor_list['skeleton']:
            try:
                self.robot_angles[motor] = self.map_angles(self.sensor_angle_range[motor], self.robot_angle_range[motor],
                                                           self.skeleton_angles[self.robot_kinect_angles[motor]])
            except:
                print('ERROR: motor ', motor, ' not connected!')
        # print self.skeleton_angles[2]
        self.move_motors()

    def move_motors(self, angles=None):
        command = CommandPosition()
        command.id = self.motor_list['full_idx']
        command.angle = self.robot_angles[self.motor_list['full']]
        print self.robot_angles[self.motor_list['full']]
        command.speed = self.motor_speeds
        # print(command)
        self.commander.publish(command)

    def map_angles(self, kinect_range, robot_range, psi):
        new_angle = robot_range[0] + (psi - kinect_range[0]) * ((robot_range[1] - robot_range[0]) / (kinect_range[1] - kinect_range[0]))
        return new_angle

# if __name__=="__main__":

motion_control = MotionControl()
motion_control.start()

logging.basicConfig(level=logging.DEBUG)

print "Listening for commands"


