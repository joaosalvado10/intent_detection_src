#!/usr/bin/env python


import rospy
import pymongo
import roslib
from pymongo import MongoClient
import gridfs
import cv2
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError





client = MongoClient("10.0.23.165", 27017)
db=client.Images
fs=gridfs.GridFS(db)
filelist=fs.list()


print filelist

filename=filelist[0]
data = fs.get_last_version(filename).read()
fout =  open(filename,"wb")
fout.write(data)
fout.close()


img = cv2.imread(filename)
#cv2.imshow('image',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

rospy.init_node('BringInAnImage',anonymous=True)

pub2 = rospy.Publisher('/BringInAnImage', Image, queue_size = 10)



print("okkkkkkkk")
while not rospy.is_shutdown():

    try:
        # talk to ROS
        filelist=fs.list()
        print filelist
        filename=filelist[0]

        data = fs.get_last_version(filename).read()
        fout =  open(filename,"wb")
        fout.write(data)
        #fout.close()

        img = cv2.imread(filename)




        bridge = CvBridge()
        pub2.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        #rate.sleep()
    except:
        print("no data available")