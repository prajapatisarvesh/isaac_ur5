#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState

joints_dict = {}


def joint_states_callback(message):
    rospy.loginfo(message.position[0])
    joint_commands = JointState()

    joint_commands.header = message.header

    for i, name in enumerate(message.name):
        joints_dict[name] = message.position[i]

    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()
    pub.publish(joint_commands)

    return


if __name__ == "__main__":
    rospy.init_node("ur5_moveit_connect")
    pub = rospy.Publisher("/joint_command", JointState, queue_size=1)
    rospy.Subscriber("/joint_command_desired", JointState, joint_states_callback, queue_size=10)
    rospy.spin()
