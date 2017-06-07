#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import pymongo
from pymongo import MongoClient
import pprint
from bson.objectid import ObjectId


from OpenFace.msg import My_message
from OpenFace.msg import pose_message_all
import sklearn

from sklearn import svm
import ast
import glob
import os







def gaze_callback(data,clf):




     vect=[data.person[0].pose_rot_x,data.person[0].pose_rot_y,data.person[0].pose_rot_z]

     pred = clf.predict(vect)
     print pred



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

if __name__ == '__main__':


    points=[]
    label=[]

    path = '/home/jorgematos/image_transport_ws/training_looking_files'


    print("training model")
    for filename in glob.glob(os.path.join(path, '*.txt')):

        file = open(filename, "r")
        head, tail = os.path.split(filename)
        print(tail)


        if(tail[0]=='n'):
            for line in file:
                line_list=ast.literal_eval(line)
                points.append(line_list)
                label.append(0)
        if(tail[0]=='l'):
            for line in file:
                line_list=ast.literal_eval(line)
                points.append(line_list)
                label.append(1)


    clf = svm.SVC()
    clf.fit(points, label)

    print("model trained")

    x=clf.predict([points[-1]])
    print(x)

    listener(clf)
