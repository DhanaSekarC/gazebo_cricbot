#!/usr/bin/env python2
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge

bridge = CvBridge()
def callback(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # cv2.waitKey(3)
    img = cv2.medianBlur(img,5)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # cv2.imshow("gray", gray)
    # cv2.waitKey(1)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
    # inverse ratio = 1
    # min dist between circles is set to 50
    # print(circles)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
	for (x, y, r) in circles:
		cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
        cv2.imshow("frame",img)
        cv2.waitKey(1)

rospy.init_node('object_detection')
sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)

rospy.spin()
