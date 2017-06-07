#!/usr/bin/env python


import rospy


from OpenFace.msg import intent_msg
from OpenFace.msg import intent_msg_all


import sys
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv_bridge import CvBridge, CvBridgeError
import glob
import copy

from  scipy  import misc

from scipy.spatial import distance
from munkres import Munkres, print_matrix


class person_intent:
 def __init__(self,px,py,pz,l,g,ri,bh,bw,bx,by,id):

    self.pose_tra_x =px
    self.pose_tra_y =py
    self.pose_tra_z =pz
    self.looking =l
    self.gesture =g
    self.result_interact =ri
    self.box_h =bh
    self.box_w =bw
    self.box_x =bx
    self.box_y =by
    self.id_model =id

    self.lifeFrame=100







class listener_intent:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/image",Image,self.callback_image)
    self.intent_sub = rospy.Subscriber("results_interaction_intent",intent_msg_all,self.callback_intent)


    self.intent_results = []
    self.intent_results_discrete = []

    self.total_persons_discrete = 0
    self.total_persons=0

    self.img_box=  cv2.imread('images/box.png')
    self.img_box = cv2.resize(self.img_box, (50, 50))
    self.img_cool=  cv2.imread('images/cool.jpg')
    self.img_cool = cv2.resize(self.img_cool, (50, 50))
    self.img_hand_shake_right =  cv2.imread('images/hand_shake_right.jpg')
    self.img_hand_shake_right = cv2.resize(self.img_hand_shake_right, (50, 50))
    self.img_hand_shake_left  =  cv2.imread('images/hand_shake_left.jpg')
    self.img_hand_shake_left = cv2.resize(self.img_hand_shake_left, (50, 50))
    self.img_hand_wave_left =  cv2.imread('images/hand_wave_left.png')
    self.img_hand_wave_left = cv2.resize(self.img_hand_wave_left, (50, 50))
    self.img_hand_wave_right =  cv2.imread('images/hand_wave_right.png')
    self.img_hand_wave_right = cv2.resize(self.img_hand_wave_right, (50, 50))
    self.img_no_gesture =  cv2.imread('images/no_gesture.png')
    self.img_no_gesture = cv2.resize(self.img_no_gesture, (50, 50))
    self.img_not_tracked =  cv2.imread('images/NotTracked.png')
    self.img_not_tracked = cv2.resize(self.img_not_tracked, (50, 50))

    self.img_eye_no = cv2.imread('images/vislab_eye_no.png')
    self.img_eye_no = cv2.resize(self.img_eye_no, (50, 50))

    self.img_eye_yes = cv2.imread('images/vislab_eye_yes.png')
    self.img_eye_yes = cv2.resize(self.img_eye_yes, (50, 50))

    self.img_yes=  cv2.imread('images/yes.jpg')
    self.img_yes = cv2.resize(self.img_yes, (50, 50))

    self.img_no =  cv2.imread('images/no.jpg')
    self.img_no = cv2.resize(self.img_no, (50, 50))

    self.cv_image_2=[]




  def callback_image(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      self.cv_image_2 = copy.deepcopy(cv_image)

    except CvBridgeError as e:
      print(e)

    try:
        #cv2.circle(cv_image, (100,470), 10, 255)

        #print(cv_image.shape)
        #480(y- baixo cima)  640(x - esquerda direita)



        for i in range(self.total_persons):



            #print ("box y")
            #print(self.intent_results[i].box_y)


            #print ("box x")
            #print(self.intent_results[i].box_x)

            #print("self.img_no.shape[0]")
            #print(self.img_no.shape[0])


            #print("the total number of persons",self.total_persons)

            ofset_y1 = -100
            ofset_x1 = -20

            ofset_y2 = ofset_y1
            ofset_x2 = ofset_x1+50

            ofset_y3 = ofset_y2
            ofset_x3 = ofset_x2 + 50

            #roi_color = frame[y:y+h, x:x+w]

            #print("the sum !!!!!!")

            #print("IN x FROM",self.intent_results[i].box_y + ofset_y1,"TO", self.intent_results[i].box_y+self.img_yes.shape[0] + ofset_y1)

            #place from where starts
            if(self.intent_results[i].box_y + ofset_y1>0 and self.intent_results[i].box_y + ofset_y1<430 ):
                #in this case everthing is correct in y axis
                pass

            if(self.intent_results[i].box_y + ofset_y1<0):
                #need to change ofset_y1
                ofset_y1=ofset_y1+400
                ofset_y2=ofset_y1
                ofset_y3=ofset_y2


            if(self.intent_results[i].box_y + ofset_y1>430):
                ofset_y1=ofset_y1-100
                ofset_y2=ofset_y1
                ofset_y3=ofset_y2


            #place from where starts
            if(self.intent_results[i].box_x  +ofset_x1>0 and  self.intent_results[i].box_x  +ofset_x3<640 ):
                #in this case everthing is correct in x axis
                pass

            if(self.intent_results[i].box_x  +ofset_x1<0):
                #need to change x offeset
                ofset_x1 =  ofset_x1+50 #(self.intent_results[i].box_x  +ofset_x1)
                ofset_x2 = ofset_x1 +50 #(self.intent_results[i].box_x  +ofset_x1)
                ofset_x3 = ofset_x2 +50 #(self.intent_results[i].box_x  +ofset_x1)

            if(self.intent_results[i].box_x  +ofset_x3 > 590):
                #need to change x offeset
                ofset_x1 =  ofset_x1 - 100 #( (self.intent_results[i].box_x  +ofset_x3)-590)
                ofset_x2 = ofset_x2 - 100 #( (self.intent_results[i].box_x  +ofset_x3)-590)
                ofset_x3 = ofset_x3 - 100 #( (self.intent_results[i].box_x  +ofset_x3)-590)





            #first image overwriting
            if(self.intent_results[i].result_interact== True):
                cv_image[self.intent_results[i].box_y + ofset_y1:  self.intent_results[i].box_y+self.img_yes.shape[0] + ofset_y1 , self.intent_results[i].box_x  +ofset_x1:self.intent_results[i].box_x+self.img_yes.shape[1]  +ofset_x1] = self.img_yes
            if(self.intent_results[i].result_interact== False):
                cv_image[self.intent_results[i].box_y + ofset_y1:  self.intent_results[i].box_y+self.img_no.shape[0] + ofset_y1 , self.intent_results[i].box_x  +ofset_x1: self.intent_results[i].box_x+self.img_no.shape[1]  +ofset_x1] = self.img_no

            #second image overwriting
            if(self.intent_results[i].looking == True):
                cv_image[self.intent_results[i].box_y + ofset_y2:  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y2 , self.intent_results[i].box_x  +ofset_x2 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x2] = self.img_eye_yes
            if(self.intent_results[i].looking == False):
                cv_image[self.intent_results[i].box_y + ofset_y2:  self.intent_results[i].box_y+self.img_eye_no.shape[0] + ofset_y2 , self.intent_results[i].box_x  +ofset_x2 :self.intent_results[i].box_x+self.img_eye_no.shape[1]  +ofset_x2] = self.img_eye_no

            #no gesture
            if(self.intent_results[i].gesture == 0):
                cv_image[self.intent_results[i].box_y + ofset_y3:  self.intent_results[i].box_y+self.img_no_gesture.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x + self.img_no_gesture.shape[1]  +ofset_x3] = self.img_no_gesture
             #handshake left
            if(self.intent_results[i].gesture == 1):
                cv_image[self.intent_results[i].box_y + ofset_y3:  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_hand_shake_left
            #handshake right
            if(self.intent_results[i].gesture == 2):
                cv_image[self.intent_results[i].box_y + ofset_y3    :  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_hand_shake_right
            #hanwave left
            if(self.intent_results[i].gesture == 3):
                cv_image[self.intent_results[i].box_y + ofset_y3    :  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_hand_wave_left
            #handwave right
            if(self.intent_results[i].gesture == 4):
                cv_image[self.intent_results[i].box_y + ofset_y3    :  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_hand_wave_right
            #cool
            if(self.intent_results[i].gesture == 5):
                cv_image[self.intent_results[i].box_y + ofset_y3    :  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_cool
            #boox
            if(self.intent_results[i].gesture == 6):
                cv_image[self.intent_results[i].box_y + ofset_y3    :  self.intent_results[i].box_y+self.img_eye_yes.shape[0] + ofset_y3  , self.intent_results[i].box_x  +ofset_x3 :self.intent_results[i].box_x+self.img_eye_yes.shape[1]  +ofset_x3] = self.img_box
    except cv2.error as e:

        print (e)
        pass

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


  def callback_intent(self,data):
    self.total_persons_discrete=data.total_models
    self.intent_results_discrete=[]

    for i in range(data.total_models):
        #print(data.intent_person[i])
        new_person= person_intent(data.intent_person[i].pose_tra_x,data.intent_person[i].pose_tra_y,data.intent_person[i].pose_tra_z,data.intent_person[i].looking,data.intent_person[i].gesture,data.intent_person[i].result_interact,int(data.intent_person[i].box_h),int(data.intent_person[i].box_w),int(data.intent_person[i].box_x),int(data.intent_person[i].box_y),data.intent_person[i].id_model)
        self.intent_results_discrete.append(new_person)
    #print self.intent_results

  def intent_continuos_update(self,discrete_intent_results):

    #this represents the actual frame
    #self.intent_results_discrete

    #this represent the last frame
    #self.intent_results


    #first we try to do the matching using a distance not above 0.5 m , if fail create new persons or decrease lifetime frame

    #first calculate the distance between them and construct the matrix

    #row corresponds to new persons
    #column corresponds to saved persons

    #first case
    if len(self.intent_results)==0:
        self.intent_results=self.intent_results_discrete[:]

    matrix=[]
    for i in range(len(self.total_persons_discrete)):
        new_row_matrix=[]
        point_new = (self.intent_results_discrete[i].pose_tra_x,self.intent_results_discrete[i].pose_tra_y,self.intent_results_discrete[i].pose_tra_z)

        for j in range(len(self.intent_results)):
          #calculate the distance

            point_detected = (self.intent_results[j].pose_tra_x,self.intent_results[j].pose_tra_y,self.intent_results[j].pose_tra_z)

            distance_cal = distance.euclidean(point_detected,point_new)
            new_row_matrix.append(distance_cal)
        matrix.append(new_row_matrix)
    if(len(matrix)>0):
        m = Munkres()
        indexes = m.compute(matrix)
        print_matrix(matrix, msg='Lowest cost through this matrix:')
        #row means detect person , column means last detected person

        #this solves the case where the number of persons is equal to last frame
        for row, column in indexes:
            value = matrix[row][column]
            total += value
            print '(%d, %d) -> %d' % (row, column, value)
            self.intent_results[column]=self.intent_results_discrete[row]

        #this solves the case where the number of persons detected is more than the last - adding person
        if len(indexes) < len(self.intent_results_discrete):
            #find the index not assign
            index_not_Assign_new_person = range(len(self.intent_results_discrete))
            for row, column in indexes:
                index_not_Assign_new_person = list(filter(lambda x: x != row, index_not_Assign_new_persont))
        


        #this solves the case where the number of persons detected is less than the last - decrease live points
        if len(indexes) < len(self.intent_results):
            #remove one life point


        #if have 0 life points then die
    else:
    #if no one was detected kill every one a bit



def main(args):
  ic = listener_intent()
  rospy.init_node('intent_listener', anonymous=True)


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':


    #image = [cv2.imread("box.png") in glob.glob("/home/jorgematos/image_transport_ws/images")]
    '''
    img_box=  cv2.imread('images/vislab_eye_yes.png')
    cv2.imshow("Image windowl", img_box)
    k = cv2.waitKey(0)
    if k == 27:         # wait for ESC key to exit
        cv2.destroyAllWindows()
    '''
    main(sys.argv)