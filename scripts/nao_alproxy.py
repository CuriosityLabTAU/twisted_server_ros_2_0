
from naoqi import ALProxy
import sys
import almath
import json
import time
import argparse


class NaoALProxy:

    def __init__(self, nao_ip=None):
        # Init the connection to nao
        self.robotIP = '192.168.0.100'
        self.port = 9559
        self.success = False

        i=1
        while ((not self.success) and i<=8):
            if nao_ip is None:
                self.robotIP = '192.168.0.10'+str(i)
            else:
                self.robotIP = nao_ip

            try:
                self.motionProxy = ALProxy("ALMotion", self.robotIP, self.port)
                self.audioProxy = ALProxy("ALAudioPlayer", self.robotIP, self.port)
                self.managerProxy = ALProxy("ALBehaviorManager", self.robotIP, self.port)
                self.postureProxy = ALProxy("ALRobotPosture", self.robotIP, self.port)
                self.trackerProxy = ALProxy("ALTracker", self.robotIP, self.port)
                self.tts = ALProxy("ALTextToSpeech", self.robotIP, self.port)
                self.success = True
                print("self.robotIP", self.robotIP)
            except Exception,e:
                print "Could not create proxy to ALMotion"
                print "Error was: ",e
                #sys.exit(1)
            i=i+1

        if (not self.success):
            sys.exit(1)

    def start_nao(self):
        self.robotConfig = self.motionProxy.getRobotConfig()  # Get the Robot Configuration
        self.motionProxy.rest()
        self.motionProxy.setStiffnesses("Body", 1.0)

    def parse_message(self, message):
        # message is json string in the form of:  {'action': 'run_behavior', 'parameters': ["movements/introduction_all_0",...]}
        # eval the action and run with parameters.
        # For example, eval result could look like: self,say_text_to_speech(['hello','how are you?'])
        message = str(message)
        # print("parse_message", message)
        message_dict = json.loads(message)
        action = str(message_dict['action'])
        if 'parameters' in message_dict:
            parameters = message_dict['parameters']
        else:
            parameters = ""
        print("PARSE_MESSAGE")
        print(str("self."+action+"("+str(parameters)+")"))
        eval(str("self."+action+"("+str(parameters)+")"))

        #print("action=",action)
        #if (action == 'say_text_to_speech'):
        #    print('say_text_to_speech')
        #    self.say_text_to_speech(parameters)
        #    #self.eval(action)(parameters)
        #if (action == 'run_behavior'):
        #    print(help(self.run_behavior))
        #    self.print_installed_behaviors()
        #    self.run_behavior(parameters)
        #if (action == 'play_audio_file'):
        #    #self.play_audio_file(parameters)
        #    self.eval(action)(parameters)

    def get_angles(self):
        names = "Body"
        use_sensors = False
        print("get_angles")
        print "Command angeles:"
        print(str(self.motionProxy.getAngles(names, use_sensors)))
        use_sensors = True
        print "Sensor angeles:"
        print(str(self.motionProxy.getAngles(names, use_sensors)))
        return self.motionProxy.getAngles(names, use_sensors)

    def run_behavior(self, parameters):
        ''' run a behavior installed on nao. parameters is a behavior. For example "movements/introduction_all_0" '''
        try:
            behavior = str(parameters[0])
            print("behavior",behavior)
            if len(parameters) > 1:
                if parameters[1] == 'wait':
                    self.managerProxy.runBehavior(behavior)

                else:
                    self.managerProxy.post.runBehavior(behavior)
            else:
                self.managerProxy.post.runBehavior(behavior)

        except Exception, e:
            print "Could not create proxy to ALMotion"
            print "Error was: ", e

    def print_installed_behaviors(self):
        # print all the behaviors installed on nao
        names = self.managerProxy.getInstalledBehaviors()
        print "Behaviors on the robot:"
        print names

    def say_text_to_speech (self, parameters):
        # make nao say the string text
        # parameters in the form of ['say something','say something','say something']
        for text in parameters:
            print("say_text_to_speech", text)
            self.tts.say (str(text))

    def play_audio_file (self, parameters):
        # Play audio file on nao.
        # For example: file = '/home/nao/wav/ask_again_0.wav'
        print(parameters)
        file = str(parameters[0])
        print(file)
        self.audioProxy.playFile(file, 1.0, 0.0)

    def print_installed_sound_sets_list(self):
        #print all the sounds installed on nao
        ssl = self.audioProxy.getInstalledSoundSetsList()
        print ssl

    def face_tracker(self, parameters):
        target_name = "Face"
        face_width = 0.1
        #add target to track.
        self.trackerProxy.registerTarget(target_name, face_width)
        #The, start tracker
        self.trackerProxy.track(target_name)
        print "ALTracker successfully started, now show your face to robot!"
        print "Use Ctrl+c to stop this script"

        try:
            while True:
                time.sleep(1)
                #print(str(self.trackerProxy.getTargetPosition()))
        except KeyboardInterrupt:
            print
            print "Interrupted by user"
            print "Stopping"

        #stop tracker
        self.trackerProxy.stopTracker()
        self.trackerProxy.unregisterAllTargets()
        self.motionProxy.rest()

    def get_target_position(self):
        target_position = self.trackerProxy.getTargetPosition()
        print(str(target_position))
        return (str(target_position))

    def rest(self):
        self.motionProxy.rest()

    def wake_up(self):
        self.motionProxy.wakeUp()

    def look_at(self, parameters):
        vect2 = parameters[0]
        fractionmaxspeed = parameters[1]
        use = parameters[2]
        self.trackerProxy.lookAt(vect2, fractionmaxspeed, use)

    def point_at(self,parameters):
        print('pointAt', parameters)
        effector = 'RArm'
        vect = parameters[0]
        fractionmaxspeed = parameters[1]
        use = parameters[2]
        self.trackerProxy.pointAt(effector, vect, fractionmaxspeed, use)

    def open_hand(self, parameters):
        print('open_hand', parameters)
        hand_name = parameters[0]
        self.motionProxy.openHand('RHand')

    def change_pose(self, data_str):
        # data_str = 'name1, name2;target1, target2;pMaxSpeedFraction'

        info = data_str.split(';')
        pNames = info[0].split(',')
        pTargetAngles = [float(x) for x in info[1].split(',')]
        # pTargetAngles = [x * almath.TO_RAD for x in pTargetAngles]  # Convert to radians
        pMaxSpeedFraction = float(info[2])
        # print(pNames, pTargetAngles, pMaxSpeedFraction)
        self.motionProxy.post.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

    def do_animation(self, action):
        # This function is from tangram. I'm leaving it here just as an example for changing poses

        if action == 'LOOKAT_TABLET':
            self.change_pose('HeadPitch;29.0;0.1')
        elif action == 'LOOKAT_CHILD':
            self.change_pose('HeadPitch;0.0;0.1')
        elif action == 'POSE_FORWARD':
            self.change_pose('HeadPitch;0.0;0.1')
        elif action == 'EXCITED':
            #self.change_pose('HeadPitch,RShoulderPitch;-10.0,-50.0;0.5')
            #self.change_pose('HeadPitch,RShoulderPitch;0.0,70.0;0.5')
            self.managerProxy.post.runBehavior("movements/raise_the_roof/raise_the_roof")
        elif action == 'LEFTRIGHTLOOKING':
            self.change_pose('HeadYaw;-50.0;0.2')
            self.change_pose('HeadYaw;50.0;0.2')
            self.change_pose('HeadYaw;0.0;0.1')
        elif action == 'HAPPY_UP':
            #self.change_pose('HeadPitch,HeadYaw,RShoulderPitch,LShoulderPitch;-10.0,-10.0,-50.0,-50.0;0.5')
            #self.change_pose('HeadPitch,HeadYaw,RShoulderPitch,LShoulderPitch;0.0,0.0,70.0,70.0;0.5')
            self.managerProxy.post.runBehavior("movements/raise_the_roof/raise_the_roof")
        elif action == 'PROUD':
            self.change_pose('RShoulderPitch,RElbowYaw;-50.0,-50.0;0.2')
            self.change_pose('RShoulderPitch,RElbowYaw;50.0,-10.0;0.2')
        elif action == 'SAD':
            self.change_pose('HeadPitch,HeadYaw;10.0,10.0;0.05')
            self.change_pose('HeadPitch,HeadYaw;0.0,0.0;0.1')

    def sound_tracker(self):
        print('sound_tracker')


if __name__ == "__main__":
    nao_alproxy = NaoALProxy()
    nao_alproxy.start_nao()
    time.sleep(3)
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/raise_the_roof/raise_the_roof']}
    # message_json = {'action': 'say_text_to_speech', 'parameters': ['hello']}
    nao_alproxy.print_installed_behaviors()
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/robotator_behaviors/TU01',"wait"]}
    #nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/robotator_behaviors/TU02',"wait"]}
    #nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/robotator_behaviors/TU05',"wait"]}
    #nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/robotator_behaviors/one_min_left',"wait"]}



    # nao_alproxy.parse_message(str(json.dumps(message_json)))
    #message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/r14', 'wait']}
    #nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/r57', 'wait']}
    # nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/r58', 'wait']}
    # nao_alproxy.parse_message(str(json.dumps(message_json)))
    # message_json = {'action': 'run_behavior', 'parameters': ['robot_facilitator-ad2c5c/r51', 'wait']}
    # nao_alproxy.parse_message(str(json.dumps(message_json)))
