#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import copy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import tf
import math

from rviz_textured_quads.msg import TexturedQuad, TexturedQuadArray



def make_tag(pose, imgNum, tagNum, scale):


    display_image = TexturedQuad()
    display_image.image = images[imgNum-1]
    display_image.pose = pose
    display_image.width = scale
    display_image.height = (scale * display_image.image.height)/display_image.image.width
    display_image.border_color = [0.0, 0.0, 0.0, 0.0]
    display_image.border_size = 0.0
    display_image.caption = 'Tag '+str(tagNum)

    return display_image



def tag_callback(msg):
    
    display_images.quads = []
    for i in range(0, len(msg.poses)):
        imgNum = i+1
	scale = 0.1
	tagNum = i+1
	display_image = make_tag(msg.poses[i], imgNum, tagNum, scale)
	display_images.quads.append(display_image)




def image_preload(num):
    for i in range(0, num):
        imgNum = i+1
	rospack = rospkg.RosPack()
	texture_path = rospack.get_path('rviz_textured_quads') + '/tests/textures/'    
	
        # All tag images are named as "tagN.jpg"
	img = cv2.imread(texture_path + 'tag{}.png'.format(imgNum),cv2.IMREAD_COLOR)
	img_msg = CvBridge().cv2_to_imgmsg(img, "bgr8")
        images.insert(i, img_msg)
    return images




if __name__ == '__main__':

    try:
            rospy.init_node('rviz_tag_display', anonymous=True)
            images = []
            images = image_preload(25)

	    pose_sub = rospy.Subscriber("/optimized_tags", PoseArray, tag_callback)
	    image_pub = rospy.Publisher("/tag_drawer", TexturedQuadArray, queue_size=10)

	    display_images = TexturedQuadArray()

	    rate = rospy.Rate(1) # 1hz


	    while not rospy.is_shutdown():

		image_pub.publish(display_images)
		rate.sleep()

    except rospy.ROSInterruptException:
        pass
