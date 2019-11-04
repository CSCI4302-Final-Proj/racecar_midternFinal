#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # racecar message
from sensor_msgs.msg import Image
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
'''

class GoBlindState(EventState):
    '''
	Driving state for a ground robot. This state allows the robot to drive forward a certain time
        at a specified velocity/ speed.

	-- speed 	float 	Speed at which to drive the robot
        -- time         float   Time at which to drive the robot

	<= failed 			    If behavior is unable to ready on time
	<= done 				Example for a failure outcome.

	'''
    def __init__(self, speed, time):
        super(GoBlindState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self.image = None
        self._speed = speed
        self._time = time

	self.vel_topic = '/cmd_vel'
        self.scan_topic = '/camera/depth/image_rect_raw'

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = ProxyPublisher({self.vel_topic: Twist_float})
        #create subscriber
        self.scan_sub = ProxySubscriberCached({self.scan_topic: Image})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)

    def execute(self, userdata):
        if not self.cmd_pub: # check if  message in self.cmd_pub to publish to /cmd_vel else we exit
            return 'failed'

        #measure distance travelled
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()

        if elapsed_time >= self._time:
            return 'done'
        #drive
        self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        Logger.loginfo("Drive FWD STARTED!")
        #set robot speed here
        self.cmd_pub = Twist_float()
        self.cmd_pub.vel = self._speed
        self.cmd_pub.angle = 0.0
        self._start_time = rospy.Time.now()

    def on_exit(self, userdata):
        self.cmd_pub.vel = 0.0
        self.cmd_pub.angle = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        Logger.loginfo("Drive FWD ENDED!")

    def on_start(self):
        Logger.loginfo("Drive FWD READY!")
        self._start_time = rospy.Time.now() #bug detected! (move to on_enter)

    def on_stop(self):
		Logger.loginfo("Drive FWD STOPPED!")

    def scan_callback(self, data):
        self.image = data
	try:	
		self.image.data = [ ord(x) for x in list(data.data)]
        except:
                pass

