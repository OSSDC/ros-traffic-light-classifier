#!/usr/bin/env python

import PIL
import sys
from PIL import Image

import message_filters
import numpy
import rospy
import collections
from light import Light
from cv_bridge import CvBridge
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from road_wizard.msg import Signals
from runtime_manager.msg import traffic_light
from sensor_msgs.msg import Image

IMAGE_DIMENSIONS = (96, 64)


class TrafficClassifier:
  RADIUS_MULTIPLIER = 6
  MAX_RADIUS = 10

  model = None
  model_path = None
  last_published_prediction = Light.UNKNOWN

  # Remove signal outliers
  detection_queue = collections.deque(maxlen=10)

  def __init__(self):
    rospy.init_node('traffic_light_classifier')

    if not rospy.has_param('~model'):
      raise StandardError("Mode param missing: Includes path to Keras model.")

    self.model_path = rospy.get_param('~model')

    roi_signal = message_filters.Subscriber('roi_signal', Signals)
    camera_image = message_filters.Subscriber('image_raw', Image)
    self.light_detected_publisher = rospy.Publisher('light_color', traffic_light, queue_size=1)
    self.roi_image = rospy.Publisher('roi_image', Image, queue_size=1)

    time_sync = message_filters.ApproximateTimeSynchronizer([roi_signal, camera_image], 5, .1)
    time_sync.registerCallback(self.detect_signal)

  def calculate_bounds(self, signal):
    xmin = sys.maxint
    xmax = -sys.maxint - 1
    ymin = sys.maxint
    ymax = -sys.maxint - 1
    radius_max = 0

    for signal in signal.Signals:
      x = signal.u
      y = signal.v
      radius_max = max(radius_max, signal.radius)
      xmin = min(xmin, x)
      xmax = max(xmax, x)
      ymin = min(ymin, y)
      ymax = max(ymax, y)

    # Filter out huge radius values
    radius_max = min(radius_max, self.MAX_RADIUS)

    return int(xmin - self.RADIUS_MULTIPLIER * radius_max), int(xmax + self.RADIUS_MULTIPLIER * radius_max), int(
      ymin - self.RADIUS_MULTIPLIER * radius_max), int(ymax + self.RADIUS_MULTIPLIER * radius_max)

  @staticmethod
  def crop_image(image, xmin, xmax, ymin, ymax):
    return image.crop((xmin, ymin, xmax, ymax))

  def predict_light(self, cropped_roi):
    # Load CNN Model
    loaded_model = self.get_model()
    image_array = img_to_array(cropped_roi.resize(IMAGE_DIMENSIONS, PIL.Image.ANTIALIAS))
    prediction = loaded_model.predict(image_array[None, :])
    if prediction[0][0] == 1:
      return Light.GREEN
    elif prediction[0][1] == 1:
      return Light.RED
    else:
      return Light.UNKNOWN

  def most_common_prediction(self, prediction):
    self.detection_queue.append(prediction)
    try:
      counter = collections.Counter(self.detection_queue)
      return counter.most_common(1)[0][0]
    except IndexError:
      return prediction

  def get_model(self):
    # TODO Fix: Must load model from ROS callback thread
    if not self.model:
      self.model = load_model(self.model_path)
    return self.model

  def detect_signal(self, signal, image):
    if len(signal.Signals) == 0:
      # No signals are visible
      self.publish_prediction(Light.UNKNOWN)
      return

    # Convert the image to PIL
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(image, "rgb8")
    image = PIL.Image.fromarray(cv_image)

    # Find the bounds of the signal
    xmin, xmax, ymin, ymax = self.calculate_bounds(signal)

    # Crop the image for the ROI
    cropped_roi = self.crop_image(image, xmin, xmax, ymin, ymax)

    self.roi_image.publish(cv_bridge.cv2_to_imgmsg(numpy.array(cropped_roi), "rgb8"))
    # Run the cropped image through the NN
    prediction = self.predict_light(cropped_roi)

    self.publish_prediction(prediction)

  def publish_prediction(self, prediction):
    # Publish the prediction
    if self.most_common_prediction(prediction) != self.last_published_prediction:
      self.light_detected_publisher.publish(traffic_light(traffic_light=prediction))
      self.last_published_prediction = prediction


if __name__ == '__main__':
  try:
    TrafficClassifier()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
