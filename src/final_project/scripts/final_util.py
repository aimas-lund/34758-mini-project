import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Point

def point_sum(a, b):
    return Point(a.x + b.x, a.y + b.y, a.z + b.z)