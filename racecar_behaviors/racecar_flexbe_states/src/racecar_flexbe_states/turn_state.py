#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
#from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # racecar message
from sensor_msgs.msg import Image
from realsense2_camera.msg import Depth

'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

'''



class TurnState(EventState):
    '''
	Turn state for a ground robot. This state allows the robot to turn at a certain angle
    at a specified velocity/ speed.

        -- t_speed 	float 	Speed at which to turn the robot
        -- turn_angle   float   The angle that the robot should make
        -- forward_dist float   free distance in front of robot

	<= failed 			    If behavior is unable to succeed on time
	<= done 				forward distance becomes sufficantly large

	'''
    def __init__(self, t_speed, turn_angle, forward_dist, timeout):
        super(TurnState, self).__init__(outcomes=['failed', 'done'])
        self._t_speed = t_speed
        self._turn_angle = turn_angle
        self._forward_dist = forward_dist
        self._timeout = timeout

        self._start_time = None
        self.depth = None

        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/depth_data'

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = ProxyPublisher({self.vel_topic: Twist_float })
        #create subscriber
        self.scan_sub = ProxySubscriberCached({self.scan_topic: Depth})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)

    def execute(self, userdata):
        if not self.cmd_pub: # check if  message in self.cmd_pub to publish to /cmd_vel else we exit
            Logger.loginfo('messesage does not exist')
            return 'failed'
        #run obstacle checks [index 0: left, 360: middle, 719: right]
        if(self.depth is not None):
            Logger.loginfo('FWD free distance is: %s' % self.depth.center)
	    Logger.loginfo('Turn angle is : %s' % self._turn_angle)
            if self.depth.center >= self._forward_dist:
                return 'done'

            #measure distance travelled
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()

            if elapsed_time >= self._timeout:
                Logger.loginfo('Reached timeout')
                return 'failed'

        #drive
        self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        Logger.loginfo("Turn RIGHT STARTED!")
        #set robot speed here
        self.cmd_pub = Twist_float()
        self.cmd_pub.vel = self._t_speed
        self.cmd_pub.angle = self._turn_angle
        self._start_time = rospy.Time.now()


    def on_exit(self, userdata):
        self.cmd_pub.vel = 0.0
        self.cmd_pub.angle = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        Logger.loginfo("Turn RIGHT ENDED!")

    def on_start(self):
        Logger.loginfo("Drive FWD READY!")
        self._start_time = rospy.Time.now() #bug detected! (move to on_enter)


    def on_stop(self):
		Logger.loginfo("Turn RIGHT STOPPED!")

    def scan_callback(self, data):
        self.depth = data

