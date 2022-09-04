#!/usr/bin/env python
## coding: UTF-8

import rospy
import rosparam
from yolov5_pytorch_ros.msg import BoundingBoxes
import math
from emcl.msg import DetectObjects, DetectObject

class yolo_to_landmark:
    def __init__(self):
        self.width = int(rospy.get_param("/yolo_to_landmark/width", "1280"))
        rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.cb_yolo)
        self.pub = rospy.Publisher("/detectobjects", DetectObjects, queue_size=1)

    def cb_yolo(self, data):
        detect_objects = DetectObjects()
        for bounding_box in data.bounding_boxes:
            detect_object = DetectObject()
            detect_object.name = bounding_box.Class
            detect_object.yaw = -(float(bounding_box.xmax + bounding_box.xmin)/2 - self.width/2) / (self.width/2) * math.pi
            detect_object.distance = 0
            #print(str(detect_object.name)+", yaw: "+str(detect_object.yaw))
            detect_objects.detectobjects.append(detect_object)
        self.pub.publish(detect_objects)

if __name__ == '__main__':
    rospy.init_node('yolo_to_landmark')
    yolo_to_landmark()
    rospy.spin()

