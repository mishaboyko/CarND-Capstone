#!/usr/bin/env python

import rospy
from styx_msgs.msg import TrafficLight

import os
import glob
import numpy as np
import cv2
import tensorflow as tf

MODEL_PATH="/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/"
class TLClassifier(object):
    def __init__(self):
        self.threshold = 0.3

        inference_graph_path = MODEL_PATH + "model/frozen_inference_graph_well-trained.pb"

        self.detection_graph = tf.Graph()

        self.load_graph(inference_graph_path)
        self.load_tensors()

    def load_graph(self, inference_graph_path):
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()

            with tf.gfile.GFile(inference_graph_path, 'rb') as f:
                serialized_graph = f.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

    def load_tensors(self):
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        self.sess = tf.Session(graph=self.detection_graph, config=config)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # part of the image where a traffic light is detected
        self.tl_detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # level of confidence for each traffic light
        self.tl_detection_scores =self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.tl_detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.tl_detection_num_detections =self.detection_graph.get_tensor_by_name('num_detections:0')

    def print_box_range(self, box, dim):
        height, width = dim[0], dim[1]
        box_range = [int(box[0] * height), int(box[1] * width), int(box[2] * height), int(box[3] * width)]
        rospy.loginfo("TL detected in range: {0}".format(box_range))
        #print("TL detected in range: {0}".format(box_range))

    def get_light_state(self, boxes, classes, scores, image_dim):
        for box, score, class_label in zip(boxes, scores, classes):
            if score > self.threshold:
                self.print_box_range(box, image_dim)

                class_label = int(class_label)
                if class_label == 1:
                    rospy.loginfo("Detected light {RED}")
                    return TrafficLight.RED
                    #return 0
                elif class_label == 2:
                    rospy.loginfo("Detected light {YELLOW}")
                    return TrafficLight.YELLOW
                    #return 1
                elif class_label == 3:
                    rospy.loginfo("Detected light {GREEN}")
                    return TrafficLight.GREEN
                    #return 2
        rospy.loginfo("Detected light {UNKNOWN}")
        return TrafficLight.UNKNOWN
        #return 4

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_dim = image.shape[0:2]

        expanded_image = np.expand_dims(image, axis=0)

        # detect Traffic lights in the input image and their color
        (boxes, scores, classes, num_detections) = self.sess.run(
            [self.tl_detection_boxes, self.tl_detection_scores, self.tl_detection_classes, self.tl_detection_num_detections],
            feed_dict={self.image_tensor: expanded_image})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes)
        scores = np.squeeze(scores)

        rospy.loginfo("Highest detection score: {0}".format(max(scores)) )
        #print("Highest detection score: {0}".format(max(scores)) )

        return self.get_light_state(boxes, classes, scores, image_dim)

if __name__ == '__main__':
    tl_cls =TLClassifier()

    test_images_path = MODEL_PATH + "test_images/*.png"

    for image_path, label in zip(sorted(glob.glob(test_images_path)), [0, 2, 2, 2, 0, 2, 1, 2]):
        image =  cv2.imread(image_path)
        traffic_light = tl_cls.get_classification(image)

        print("Image: {}".format(image_path))
        print("Detected: {}, labeled {}".format(traffic_light, label))