import visualization_utils as vis_util
import os
import cv2
import time
import calendar


class ImageSaver(object):
    def __init__(self, category_index, output_dir):
        self.category_index = category_index
        self.output_dir = output_dir

    def save_visualization(self, image, boxes, classes, scores, max_state):
        timestamp = calendar.timegm(time.gmtime())
        output_dir = os.path.join(self.output_dir, 'camera', max_state)
        self._save_image(image, timestamp, output_dir)
        visualized_image = self._visualize_detections(image, boxes, classes, scores)
        output_dir = os.path.join(self.output_dir, 'visualization', max_state)
        self._save_image(visualized_image, timestamp, output_dir)

    def _visualize_detections(self, image, boxes, classes, scores):
        # Visualization of the results of a detection.
        return vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            boxes,
            classes,
            scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=4)

    @staticmethod
    def _save_image(image, timestamp, output_dir):
        # Use timestamp to make unique filenames
        file_name = "image_{}.jpg".format(timestamp)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        pathname = os.path.join(output_dir, file_name)
        output_image = image[:, :, ::-1]
        cv2.imwrite(pathname, output_image)
        print("image {} saved".format(pathname))

    def save_image(self, image, state):
        # Use timestamp to make unique filenames
        timestamp = calendar.timegm(time.gmtime())
        output_dir = os.path.join(self.output_dir, state)
        self._save_image(image, timestamp, output_dir)
