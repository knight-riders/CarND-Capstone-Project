from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import cv2
import numpy as np
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

#PATH_TO_CKPT = '../../../tl_detector/fine_tuned_sim_model/frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self, model_dir = None):

        ## Get model directory and check model files
        if model_dir is None:
            import rospkg
            rp = rospkg.RosPack()
            model_dir = os.path.join(rp.get_path('tl_detector'), 'model')
        rospy.loginfo('Using model directory {}'.format(model_dir))

        detection_model_path = os.path.join(model_dir, 'model_detection.pb')
        if not os.path.exists(detection_model_path):
            rospy.logerr('Detection model not found at {}'.format(detection_model_path))

        classification_model_path = os.path.join(model_dir, 'model_classification.pb')
        if not os.path.exists(classification_model_path):
            rospy.logerr('Classification model not found at {}'.format(classification_model_path))

        # Activate optimizations for TF
        if os.getenv('HOSTNAME') == 'miha-mx':
            self.config = tf.ConfigProto(device_count = {'GPU': 1, 'CPU': 1}) # log_device_placement=True
            self.config.gpu_options.allow_growth = True
            self.config.gpu_options.per_process_gpu_memory_fraction = 1
        else:
            self.config = tf.ConfigProto()
        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level

        # Load graphs
        self.graph_detection = _load_graph(detection_model_path, self.config)
        self.graph_classification = _load_graph(classification_model_path, self.config)

        # Create TF sessions
        self.sess_detection = tf.Session(graph=self.graph_detection, config=self.config)
        self.sess_classification = tf.Session(graph=self.graph_classification, config=self.config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.graph_detection.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected
        self.detection_boxes = self.graph_detection.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects
        self.detection_scores = self.graph_detection.get_tensor_by_name('detection_scores:0')

        # This is the class from MS COCO dataset, we only need class 10 = traffic light
        self.detection_classes = self.graph_detection.get_tensor_by_name('detection_classes:0')

        # Get input and output tensors for the classification
        self.in_graph = self.graph_classification.get_tensor_by_name('input_1_1:0')
        self.out_graph = self.graph_classification.get_tensor_by_name('output_0:0')

        # Model index to TLD message
        self.index2msg = {0: TrafficLight.RED, 1: TrafficLight.GREEN, 2: TrafficLight.YELLOW}
        self.index2color = {0: (255, 0, 0), 1: (0, 255, 0), 2: (255, 255, 0)}

        # TL publisher
        self.bridge = CvBridge()
        self.traffic_light_pub = rospy.Publisher('/tld/traffic_light', Image, queue_size = 1)

        ## kick classification to preload models
        self.detection(cv2.cvtColor(np.zeros((600, 800), np.uint8), cv2.COLOR_GRAY2RGB))
        self.classification(cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB))

    def get_classification(self, image):
        with Timer('get_classification'):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            box = self.detection(image)
            if box == None:
                if rospy.get_param('~publish_traffic_light', False):
                    self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB), "rgb8"))
                return TrafficLight.UNKNOWN

            left, right, top, bottom = box
            img_crop = image[top:bottom, left:right]
            traffic_light = cv2.resize(img_crop, (32, 32))

            return self.classification(traffic_light)

    def detection(self, image):
        """ Traffic light detection """
        im_height, im_width, _ = image.shape
        image_expanded = np.expand_dims(image, axis=0)

        with self.sess_detection.as_default(), self.graph_detection.as_default(), Timer('detection'):
            boxes, scores, classes = self.sess_detection.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_expanded})

            # Extract box and append to list
            return _extractBox(boxes, scores, classes, 0.1, im_width, im_height)

    # Classification
    def classification(self, image):
        """ Traffic light classification """
        with self.sess_classification.as_default(), self.graph_classification.as_default(), Timer('classification'):
            sfmax = list(self.sess_classification.run(tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [image]}))))
            sf_ind = sfmax.index(max(sfmax))

            ## add a colored bbox and publish traffic light if needed
            ## rosparam set /tl_detector/publish_traffic_light true
            if rospy.get_param('~publish_traffic_light', False):
                cv2.rectangle(image, (0, 0), (31, 31), self.index2color[sf_ind], 1)
                self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))

        return self.index2msg[sf_ind]

if __name__ == "__main__":
    classifier = TLClassifier(model_dir = 'model')
    classifier.publish_traffic_light = False
    images_dir = 'images'

        
