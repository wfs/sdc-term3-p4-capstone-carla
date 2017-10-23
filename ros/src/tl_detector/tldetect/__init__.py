import numpy as np
import tensorflow as tf
import numpy as np
import tensorflow as tf
from styx_msgs.msg import TrafficLight
import rospy
import time


class predictor(object):
    def __init__(self, modelpath="./FrozenSyam.pb"):
        """ Loads the default model if none specficied. """
        self.modelpath = modelpath

        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.modelpath, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.graph = detection_graph
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes_tensor = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores_tensor = detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes_tensor = detection_graph.get_tensor_by_name('detection_classes:0')

        self.graph.as_default()
        self.sess = tf.Session(graph=self.graph)

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    def predict(self, image_np):

        image_np_expanded = np.expand_dims(image_np, axis=0)
        # start = rospy.get_time()
        time0 = time.time()
        (boxes, scores, classes) = self.sess.run(
            [self.boxes_tensor, self.scores_tensor, self.classes_tensor],
            feed_dict={self.image_tensor: image_np_expanded})

        end = rospy.get_time()
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        pred = list()
        skores = list()

        if len(scores) < 1:
            return TrafficLight.UNKNOWN

        for i in range(boxes.shape[0]):
            score = scores[i]
            if score > 0.3:
                pred.append(classes[i])
                skores.append(score)

        # RED, YELLOW, GREEN = 1, 2, 3

        light = TrafficLight.UNKNOWN
        if (3 in pred):
            light = TrafficLight.GREEN
        elif (2 in pred):
            light = TrafficLight.YELLOW
        else:
            light = TrafficLight.RED

        return light
