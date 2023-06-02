#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge 
import numpy as np

class ColorSensorConverter:
    def __init__(self):
        rospy.init_node('colorsensor_converter', anonymous=True)
        rospy.Subscriber("input_image", Image, self.callback)
        self.pub = rospy.Publisher('output_sensor_data', ColorRGBA, queue_size=1)
        self.bridge = CvBridge()

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        send_data = ColorRGBA()
        send_data.r = np.mean(cv_image[:,:,2])/255.0
        send_data.g = np.mean(cv_image[:,:,1])/255.0
        send_data.b = np.mean(cv_image[:,:,0])/255.0
        self.pub.publish(send_data)
 
def listener():
    cc = ColorSensorConverter()
    rospy.spin()

if __name__ == '__main__':
    listener()
