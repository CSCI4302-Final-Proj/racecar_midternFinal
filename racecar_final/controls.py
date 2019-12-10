#!/usr/bin/env python
import rospy 
#from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # implement msg  
from realsense2_camera.msg import Depth
from sensor_msgs.msgs import LaserScan



class drive_forward:
    def __init__(self):
        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/scan'
        self.cmd_pub = Twist_float()
        self.angle_diff_prev = 0
        self.kp = 0.01
        self.kd = 0.01
        self.scale = 0
         
        #create publisher passing it the vel_topic_name and msg_type
        self.pub = rospy.Publisher(self.vel_topic, Twist_float, queue_size = 10)

        #create subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic, Depth, self.scan_callback)


    def scan_callback(self, data):
        self.depth = data
        #self.depth.right = self.depth.right + 500
        #rospy.loginfo('45 deg distance left: %s, center: %s, right: %s' % (self.depth.left, self.depth.center, self.depth.right))
        
        #if angle_diff < 0:
        #	rospy.loginfo('Turn Right')
        #else:
        #	rospy.loginfo('Turn Left')
        self.left = self.data[0:200]/mean(self.data[[0:200]])
        self.right = self.data[300:340]/mean(self.data[[435:635]])
        self.center = self.data[0:200]/mean(self.data[[0:200]])

        # control scheme
        self.max_angle = 0.1
        self.angle_diff = self.left-self.right
        if self.angle_diff > 0:
            self.turn_left = True
        else:
            self.turn_right = True

        self.prop = self.kp * self.angle_diff
        self.deriv = self.kd * (self.angle_diff_prev - self.angle_diff)
        self.angle_diff_prev = self.angle_diff

        self.cmd_pub.angle = (self.prop + self.deriv)*self.scale

        self.cmd_pub.vel = -0.08
        self.pub.publish(self.cmd_pub)
        rospy.sleep(0.1)

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive_forward()
	rospy.spin()
