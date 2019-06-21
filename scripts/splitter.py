#!/usr/bin/env python
# license removed for brevity

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    try:
      image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    image_size = 640
    front = image[0:image_size, 0:image_size]
    back  = image[0:image_size, image_size:2*image_size]
    image_f_message = bridge.cv2_to_imgmsg(front, encoding="bgr8")
    image_b_message = bridge.cv2_to_imgmsg(back,  encoding="bgr8")
    image_f_message.header.stamp = data.header.stamp
    image_b_message.header.stamp = data.header.stamp
    pub_f.publish(image_f_message)
    pub_b.publish(image_b_message)


if __name__ == '__main__':
    rospy.init_node('splitter', anonymous=True)
    bridge = CvBridge()
    sub = rospy.Subscriber("image_raw", Image, callback)
    pub_f = rospy.Publisher('image_front_raw', Image, queue_size=10)
    pub_b = rospy.Publisher('image_back_raw',  Image, queue_size=10)

    rospy.spin()


