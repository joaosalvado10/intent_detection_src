#!/usr/bin/env python

import sklearn


from sklearn import svm
import sys



import glob
import os
import ast

def main():



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


    scores = cross_val_score(clf, points, label, cv=5)
    print(scores)




if __name__ == "__main__":
    main()