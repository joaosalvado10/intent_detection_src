#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import pymongo
from pymongo import MongoClient
import pprint
from bson.objectid import ObjectId

import sys


from OpenFace.msg import My_message
from OpenFace.msg import pose_message_all
import sklearn

from sklearn import svm
import ast
import os.path







def gaze_callback(data,clf):




     vect=[data.person[0].pose_rot_x,data.person[0].pose_rot_y,data.person[0].pose_rot_z]
     s=str(vect)
     clf.write(s + '\n')




def listener(clf):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('intention_detector', anonymous=True)

    rospy.Subscriber('pose_gaze',pose_message_all , gaze_callback,clf)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()







if __name__ == "__main__":



    save_path = '/home/jorgematos/image_transport_ws/training_looking_files'

    print("the name of the file has to start with noloking or loking")
    name_of_file = raw_input("What is the name of the file: ")

    completeName = os.path.join(save_path, name_of_file+".txt")

    clf = open(completeName, "w")

    listener(clf)



