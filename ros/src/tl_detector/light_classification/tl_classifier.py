from styx_msgs.msg import TrafficLight
import numpy as np
import os
import tensorflow as tf
import rospy
import yaml

from utils import label_map_util
from utils.imagesaver import ImageSaver


class TLClassifier(object):

    def __init__(self, save_visualizations):
        config_string = rospy.get_param("/traffic_light_config")

        config = yaml.safe_load(config_string)
        is_site = config['is_site']

        #
        self.MIN_SCORE_THRESHOLD = 0.50

        # saves to every original image from the camera an image annotated with the detected traffic lights
        # only needed to check how good the traffic light detection works
        self.save_visualizations = save_visualizations

        output_dir = './data'

        # directory in which the trained models are stored
        model_dir = os.path.dirname(os.path.realpath(__file__))

        # load site or simulator model dependent on the site configuration
        if is_site:
            model_file_path = os.path.join(model_dir, 'ud_capstone_site_graph.pb')
            rospy.loginfo("Site environment: {}".format(model_file_path))
            output_dir = os.path.join(output_dir, 'site')
        else:
            model_file_path = os.path.join(model_dir, 'ud_capstone_simulator_graph.pb')
            rospy.loginfo("Simulator environment: {}".format(model_file_path))
            output_dir = os.path.join(output_dir, 'simulator')

        # Load Tensorflow model graph
        self.graph = tf.Graph()
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_file_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Load label map
        label_map_file_path = os.path.join(model_dir, 'label_map.pbtxt')
        label_map = label_map_util.load_labelmap(label_map_file_path)
        categories = label_map_util.convert_label_map_to_categories(label_map,
                                                                    max_num_classes=len(label_map.item),
                                                                    use_display_name=True)

        self.category_index = label_map_util.create_category_index(categories)
        self.image_saver = ImageSaver(self.category_index, output_dir)

    def detect_traffic_lights(self, image):
        with self.graph.as_default():
            with tf.Session(graph=self.graph) as sess:
                image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
                boxes = self.graph.get_tensor_by_name('detection_boxes:0')
                scores = self.graph.get_tensor_by_name('detection_scores:0')
                classes = self.graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.graph.get_tensor_by_name('num_detections:0')

                # images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image, axis=0)
                (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],
                                                                    feed_dict={image_tensor: image_np_expanded})

                return boxes, scores, classes, num_detections

    def predict_state(self, image):
        boxes, scores, classes, num_detections = self.detect_traffic_lights(image)

        # only for debugging
        # rospy.loginfo("num_detections: {}".format(num_detections))
        # rospy.loginfo("boxes: {}".format(boxes))
        # rospy.loginfo("scores: {}".format(scores))

        normalized_scores = {"Green": 0, "Red": 0, "Yellow": 0, "Unknown": 0}

        det_count = 0
        det_indexes = []

        for i in range(0, num_detections):
            if scores is not None:
                score = scores[0][i]
                if score > self.MIN_SCORE_THRESHOLD:
                    det_state = classes[0][i]
                    det_name = self.category_index[det_state]['name']
                    normalized_scores[det_name] += score
                    det_indexes.append(i)
                    det_count += 1

        if det_count > 0:
            boxes = np.array(boxes[0][det_indexes])
            scores = np.array(scores[0][det_indexes])
            classes = np.squeeze(np.array(classes[0][det_indexes])).astype(int)

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
        rospy.loginfo("Predicted state: {} ({})".format(max_state, max_score))

        # save visualized image if the option is set
        if self.save_visualizations and det_count > 0:
            self.image_saver.save_visualization(image, boxes, classes, scores, max_state)

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
