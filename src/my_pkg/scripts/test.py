#!/usr/bin/env python3
import time
import sys
from typing import List, Tuple

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from frenet_utils import FrenetConverter
