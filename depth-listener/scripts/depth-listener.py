#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def callback(msg):
    listTest = list(msg.data)
    print(listTest)
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()