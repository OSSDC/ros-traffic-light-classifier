#!/usr/bin/env python

import rospy
import rospkg
import cv2
import os

from light import Light
from cv_bridge import CvBridge
from runtime_manager.msg import traffic_light
from sensor_msgs.msg import Image


class TrafficLightViz:
  def __init__(self):
    rospy.init_node('traffic_light_viz')
    self.path = rospkg.RosPack().get_path('traffic_light_classifier')
    self.traffic_viz_publisher = rospy.Publisher('traffic_light_viz', Image, queue_size=1)
    self.green_image = self.load_image('green.jpg')
    self.red_image = self.load_image('red.jpg')
    self.unknown_image = self.load_image('unknown.jpg')
    self.cv_bridge = CvBridge()
    rospy.Subscriber('light_color', traffic_light, self.traffic_light_callback)

  def traffic_light_callback(self, traffic_light):
    detected_signal = traffic_light.traffic_light
    if detected_signal == Light.RED:
      self.publish_image(self.red_image)
    elif detected_signal == Light.GREEN:
      self.publish_image(self.green_image)
    else:
      self.publish_image(self.unknown_image)

  def publish_image(self, image):
    self.traffic_viz_publisher.publish(self.cv_bridge.cv2_to_imgmsg(image, 'bgr8'))

  def load_image(self, name):
    return cv2.imread(os.path.join(self.path, 'light_images', name))


if __name__ == '__main__':
  TrafficLightViz()
  rospy.spin()
