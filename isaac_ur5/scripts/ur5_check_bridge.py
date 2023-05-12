#!/usr/bin/python3
'''
Program for testing ROS Bridge with UR5 model in Isaac Sim
author - @sarvesh
email - sarvesh101p@gmail.com
Referecnce - Isaac documentation and tutorials
'''

#imports for the program
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time


class ur5_check_bridge():
    '''
    Class based on ROS middle-ware
    Functions
        -> shutdown
        -> main
    '''
    def __init__(self):
        '''
        Constructor that creates ros node with name `ur5_isaac_test`
        Defines default and limits of joint poses
        '''
        rospy.init_node("ur5_isaac_test", anonymous=True)
        self.pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.name = ["elbow_joint", "wrist_1_joint", "shoulder_lift_joint", \
                                "wrist_2_joint", "shoulder_pan_joint", "wrist_3_joint"]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = np.array([0.0] * self.num_joints)
        self.default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5
        self.time_start = time.time()
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('[+] Node will start in 2 second, press ctrl+c to kill')
        rospy.sleep(2)


    def shutdown(self):
        '''
        Function - Shutdown
        Resets the ur5 to default pose when ctrl+c is pressed
        '''
        self.joint_state.position = self.default_joints
        self.pub.publish(self.joint_state)
        rospy.loginfo('[+] Node killed, Reseting to default position')
    

    def main(self):
        '''
        Function - main
        Manipulates UR5's position
        '''
        while not rospy.is_shutdown():
            self.joint_state.position = np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
            self.pub.publish(self.joint_state)
            self.rate.sleep()


if __name__ == '__main__':
    '''
    Entry point of program that utilized ur5_check_bridge class
    '''
    ur5 = ur5_check_bridge()
    try:
        ur5.main()
    except rospy.ROSInterruptException as e:
        pass