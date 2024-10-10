'''lab4_publisher.py

This code will publish a simple
go or stop command based on keyboard
input.

'''



# Special imports
#import warnings
#warnings.filterwarnings('ignore')
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior( )
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib import pyplot as plt
from PIL import Image

# Enviromental imports
#%matplotlib inline
import sys
import os
import cv2
# Append your Tensorflow object detection and darkflow directories to your path
sys.path.append('~anaconda3/envs/tf_env/Lib/site-packages/object_detection') # ~/tensorflow/models/research/object_detection
#sys.path.append('PATH_TO_DARKFLOW_FOLDER') # ~/darkflow
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Quanser imports
from pal.products.qcar import QCarRealSense
import time



#MODEL_NAME = 'faster_rcnn_inception_resnet_v2_atrous'
# MODEL_NAME = 'faster_rcnn_resnet_101'
# MODEL_NAME = 'faster_rcnn_resnet50'
# MODEL_NAME = 'faster_rcnn_inception_v2'
# MODEL_NAME = 'rfcn_resnet101'
# MODEL_NAME = 'ssd_inception_v2'
MODEL_NAME = 'ssd_mobilenet_v1'


# Path to frozen detection graph. This is the actual model that is used for the traffic sign detection.
MODEL_PATH = os.path.join('models', MODEL_NAME)
PATH_TO_CKPT = os.path.join(MODEL_PATH,'inference_graph/frozen_inference_graph.pb')

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('gtsdb_data', 'gtsdb3_label_map.pbtxt')

NUM_CLASSES = 3

# Gamepad Node
class Detector():

    def __init__(self):

        imageWidth = 1280
        imageHeight = 720

        self.IMAGE_SIZE = (20, 20)

        # Connect to remote controller & car hardware
        self.camera = QCarRealSense(mode='RGB, Depth', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)


        # prepare pretrained model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                self.serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(self.serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

        # load label map
        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)
        print(self.label_map)


        self.perform_detection()


    # helper function
    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    # Detection system
    def perform_detection(self):
        with tf.Session(graph=self.detection_graph) as sess:
            
            while True:
                sign_detected = False
                # Reset with every iteration


                # Read camera data
                self.camera.read_RGB()
                img = self.camera.imageBufferRGB
                #img = Image.open("C:/Users/user/Desktop/LAB4-pretrained/traffic-sign-detection/test_images/image1.jpg")



                image_np = img#self.load_image_into_numpy_array(img)
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


                (boxes, scores, classes, num_detections) = sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                # Visualization of the results of a detection.
                vis_util.visualize_boxes_and_labels_on_image_array(
                    image_np,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    self.category_index,
                    use_normalized_coordinates=True,
                    line_thickness=6)
                plt.figure("Stream", figsize=self.IMAGE_SIZE)
                plt.axis('off')
                plt.imshow(image_np)


                cv2.imshow('Stream', image_np)


                msg = "None"
                # Sort detections
                if num_detections == 1:
                    for detection in scores:
                        msg = str(detection)
                elif num_detections > 1:       
                    for detection in scores:
                        msg = str(detection)
                    


                # Publish drive command
                #print(msg)
                cv2.waitKey(0)

                time.sleep(0.01)
        


    # Safely disconnect hardware
    def destroy(self):
        self.camera.terminate()
    

if __name__ == '__main__':
    Detector()
