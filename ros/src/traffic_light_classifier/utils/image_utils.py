import sys

from PIL import Image
from road_wizard.msg import Signals
from sensor_msgs.msg import Image


RADIUS_MULTIPLIER = 6

def calculate_bounds(signal):
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

  return int(xmin - RADIUS_MULTIPLIER * radius_max), int(xmax + RADIUS_MULTIPLIER * radius_max), int(
    ymin - RADIUS_MULTIPLIER * radius_max), int(ymax + RADIUS_MULTIPLIER * radius_max)


def crop_image(image, xmin, xmax, ymin, ymax):
  return image.crop((xmin, ymin, xmax, ymax))

def crop_signal(signal, image):
  # Find the bounds of the signal
  xmin, xmax, ymin, ymax = calculate_bounds(signal)

  # Crop the image for the ROI
  cropped_roi = crop_image(image, xmin, xmax, ymin, ymax)

  return cropped_roi