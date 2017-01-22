import sys
from collections import defaultdict

import rospy

from PIL import Image
from itertools import groupby

from road_wizard.msg import Signals
from sensor_msgs.msg import Image

RADIUS_MULTIPLIER = 6
MAX_RADIUS = 10


def point_distance_squared(current_position, signal_position):
  x = current_position.pose.position.x - signal_position.x
  y = current_position.pose.position.y - signal_position.y
  z = current_position.pose.position.z - signal_position.z
  return x * x + y * y + z * z


def find_nearest_signal(signals, current_pose):
  signal_groups = defaultdict(list)
  distances = {}

  # Group the signals based on pole id
  for signal in signals.Signals:
    signal_groups[signal.plId].append(signal)

  # Find the group of poles closest to the current position
  for key, group in signal_groups.iteritems():
    first_signal = group[0]
    distances[first_signal.plId] = point_distance_squared(current_pose, first_signal)

  closest_pole_id = min(distances, key=distances.get)
  return signal_groups[closest_pole_id]


def calculate_bounds(signal):
  xmin = sys.maxint
  xmax = -sys.maxint - 1
  ymin = sys.maxint
  ymax = -sys.maxint - 1
  radius_max = 0

  for light in signal:
    x = light.u
    y = light.v
    radius_max = max(radius_max, light.radius)
    xmin = min(xmin, x)
    xmax = max(xmax, x)
    ymin = min(ymin, y)
    ymax = max(ymax, y)

    # Filter out huge radius values
    radius_max = min(radius_max, MAX_RADIUS)

  return int(xmin - RADIUS_MULTIPLIER * radius_max), int(xmax + RADIUS_MULTIPLIER * radius_max), int(
    ymin - RADIUS_MULTIPLIER * radius_max), int(ymax + RADIUS_MULTIPLIER * radius_max)


def crop_image(image, xmin, xmax, ymin, ymax):
  return image.crop((xmin, ymin, xmax, ymax))


def crop_signal(signals, image, current_pose):
  signal = find_nearest_signal(signals, current_pose)

  # Find the bounds of the signal
  xmin, xmax, ymin, ymax = calculate_bounds(signal)

  # Crop the image for the ROI
  cropped_roi = crop_image(image, xmin, xmax, ymin, ymax)

  return cropped_roi
