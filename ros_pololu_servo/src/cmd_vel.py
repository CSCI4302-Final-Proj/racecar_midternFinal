#!/usr/bin/env python
import rospy
# subscribe msg to float32
from racecar_flexbe_states.msg import Twist_float
# publish msg driving messages
from ros_pololu_servo.msg import MotorCommand

'''
string joint_name
float64 position
float32 speed
float32 acceleration
'''

class publish_driving(object): # needed for python 2 , implicit in python 3
    def __init__(self, sub_topic_name = '/cmd_vel'):
        # publisher
        self._drive_publisher = rospy.Publisher('/pololu/command', MotorCommand, queue_size=1)
        # subscriber
        self._sub_topic_name = sub_topic_name
        self._twist_subscriber = rospy.Subscriber( self._sub_topic_name, Twist_float, self.topic_callback)

        # get message
        self._drive_msg_motor = MotorCommand()
	self._drive_msg_steering = MotorCommand()
        self._twist_msg = Twist_float()


    def topic_callback(self, msg):
        self._twist_msg = msg
        rospy.loginfo(self._twist_msg)


        self.pub_drive(self._twist_msg)



    def get_angle(self):
        '''
        returns newewst angle data
        float32 data
        '''

        return self._angle_msg


    def pub_drive(self, msg):

        self._drive_msg_motor.joint_name = 'motor'
	self._drive_msg_steering.joint_name = 'steering'

	self._drive_msg_motor.position = msg.vel 

        self._drive_msg_steering.position = msg.angle

        self._drive_publisher.publish(self._drive_msg_motor)
	#rospy.sleep(0.1)
	self._drive_publisher.publish(self._drive_msg_steering)



if __name__ == '__main__':
    rospy.init_node('cmd_vel_node' , log_level=rospy.INFO)
    publish_driving()
    rospy.spin()


