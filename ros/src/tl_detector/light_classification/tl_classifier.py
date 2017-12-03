'''
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
'''


from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np
import os

class TLClassifier(object):
    def __init__(self):
        #load classifier
        print("TLClassifier dir: ", os.getcwd())
        self.model = load_model('light_classification/trained_model.h5')
        print(self.model.summary())
        # Test model
        dummy_image = np.zeros((32,32,3))
        print(self.model.predict(np.array([dummy_image])))
        print(self.get_classification(dummy_image))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        pred = self.model.predict(np.array([image]))[0]
        light_state = np.argmax(pred)

        if light_state == 0:
            return TrafficLight.RED
        if light_state == 1:
            return TrafficLight.YELLOW
        if light_state == 2:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN
