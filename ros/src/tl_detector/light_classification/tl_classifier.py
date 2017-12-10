from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf

PATH_TO_CKPT = '../../../tl_detector/fine_tuned_sim_model/frozen_inference_graph.pb'


class TLClassifier(object):

    def __init__(self):
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            with tf.Session(graph=self.detection_graph) as sess:
                # Define input and output Tensors for self.detection_graph
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represents level of confidence for each object.
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                self.sess = sess

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_expanded = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores,
                                                      self.detection_classes, self.num_detections],
                                                      feed_dict={self.image_tensor: image_expanded})

        class_vals = []
        state = TrafficLight.UNKNOWN

        for idx, score in enumerate(scores[0]):
            if score > 0.3:
                class_vals.append(classes[0][idx])

        if len(class_vals) > 0:
            class_val = max(class_vals)
            if class_val == 1:
                state = TrafficLight.RED
            elif class_val == 2:
                state = TrafficLight.YELLOW
            elif class_val == 3:
                state = TrafficLight.GREEN

        return state
