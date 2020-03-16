#!/usr/bin/env python2

from sensor_msgs.msg import JointState
#from sensor_msgs.msg import *
from edo_connect4.srv import *
import rospy


def callback(data):

    print("IN THE CALLBACK")

    #val = data.position[4]

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", val)


def get_joint_vals(req):

    print("GOT HERE")

    rospy.Subscriber("/edo/joint_states", JointState, callback)

    rospy.spin()

    return GetJointStatesResponse(0.0, 1.0, 0.0, 1.0, 0.0, 1.0)


def joint_server():

    rospy.init_node('return_joint_state_server')

    rospy.Service('return_joint_state', GetJointStates, get_joint_vals)

    print("Ready to return joints state.")

    rospy.spin()


if __name__ == '__main__':
    joint_server()
