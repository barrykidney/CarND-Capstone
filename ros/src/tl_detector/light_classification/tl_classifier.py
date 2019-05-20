from styx_msgs.msg import TrafficLight
import numpy as np
import os
import tensorflow as tf
import rospy
import time
import calendar
import cv2
import yaml

from utils import label_map_util
from utils import visualization_utils as vis_util


class TLClassifier(object):

    def __init__(self, save_visualizations):
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.safe_load(config_string)
        # self.img_width = self.config['camera_info']['image_width']
        # self.img_height = self.config['camera_info']['image_height']
        # rospy.loginfo("width: {} height: {}".format(self.img_width, self.img_height))
        self.is_site = self.config['is_site']
        self.save_visualizations = save_visualizations

        self.MIN_SCORE_THRESHOLD = 0.50
        self.NUM_CLASSES = 4

        self.output_dir = './data'

        model_dir = os.path.dirname(os.path.realpath(__file__))

        # load site or simulator model dependent on the current configuration
        if self.is_site:
            model_file_path = os.path.join(model_dir, 'ud_capstone_site_graph.pb')
            rospy.loginfo("Site environment: {}".format(model_file_path))
        else:
            model_file_path = os.path.join(model_dir, 'ud_capstone_simulator_graph.pb')
            rospy.loginfo("Site environment: {}".format(model_file_path))

        label_map_file_path = os.path.join(model_dir, 'label_map.pbtxt')

        print (model_file_path)

        # Load Tensorflow model graph
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_file_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Load label map
        self.label_map = label_map_util.load_labelmap(label_map_file_path)
        categories = label_map_util.convert_label_map_to_categories(self.label_map,
                                                                    max_num_classes=self.NUM_CLASSES,
                                                                    use_display_name=True)

        self.category_index = label_map_util.create_category_index(categories)

    def detect_traffic_lights(self, image):
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image, axis=0)
                (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],
                                                                    feed_dict={image_tensor: image_np_expanded})

                return boxes, scores, classes, num_detections

    def predict_state(self, image):
        boxes, scores, classes, num_detections = self.detect_traffic_lights(image)

        normalized_scores = {"Green": 0, "Red": 0, "Yellow": 0, "Unknown": 0}

        det_count = 0
        rospy.loginfo("num_detections: {}, boxes: {}".format(num_detections, boxes.shape[0]))

        for i in range(0, num_detections):
            if scores is not None:
                score = scores[i]
                if score > self.MIN_SCORE_THRESHOLD:
                    det_state = classes[i]
                    det_name = self.category_index[det_state]['name']
                    normalized_scores[det_name] += score
                    det_count += 1

        max_score = 0
        max_state = "Unknown"
        for key in normalized_scores.keys():
            # Normalize scores
            if det_count > 0:
                normalized_scores[key] /= (det_count * 1.0)
            # Get highest score
            if normalized_scores[key] > max_score:
                max_score = normalized_scores[key]
                max_state = key

        rospy.loginfo(normalized_scores)
        rospy.loginfo("Predicted state: {}   Normalized score: {}".format(max_state, max_score))

        if self.save_visualizations:
            timestamp = calendar.timegm(time.gmtime())
            self.save_image(image, timestamp, max_state, False)
            visualized_image = self.visualize_detections(image, boxes, scores, classes)
            self.save_image(visualized_image, timestamp, max_state, True)

        return max_state

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image_np = np.asarray(image, dtype="int32")
        state = self.predict_state(image_np)

        if state == "Red":
            return TrafficLight.RED
        elif state == "Yellow":
            return TrafficLight.YELLOW
        elif state == "Green":
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN

    def visualize_detections(self, image, boxes, classes, scores):
        # Visualization of the results of a detection.
        return vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=8)

    def save_image(self, image, timestamp, state, visualize=False):
        # use timestamp to make unique filenames
        file_name = "image_{}.jpg".format(timestamp)
        directory = os.path.join(self.output_dir, 'visualization' if visualize else 'camera', state)
        if not os.path.exists(directory):
            os.makedirs(directory)

        pathname = os.path.join(directory, file_name)
        output_image = image[:, :, ::-1]
        cv2.imwrite(pathname, output_image)
