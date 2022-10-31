#!/usr/bin/env python

import rospy
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped, Pose
from broadcast_marker_tf import Broadcaster

class FrameHandler:

    _DEFAULT_BASE_FRAME = "/map"
    _DEFAULT_FOREIGN_FRAME = "/marker_frame"
    _NODE_NAME = 'tf_listener'

    def __init__(self):
        rospy.init_node(self._NODE_NAME)
        self.listener = TransformListener()

    def convert_frame(self, base_frame=_DEFAULT_BASE_FRAME, foreign_frame=_DEFAULT_FOREIGN_FRAME):
        print("Trying to find tranform between the frames {} and {}".format(base_frame, foreign_frame))
        if not (self.listener.frameExists(base_frame) and self.listener.frameExists(foreign_frame)):
            print("Provided frames does not exist.")
            return
        
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform(foreign_frame, base_frame, rospy.Time(0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
        
        """p = PoseStamped()
        p.header.frame_id = foreign_frame.replace('/', '')
        p.pose.orientation.w = 1.0
        p_transformed = self.listener.transformPose(base_frame, p)
        """
        return p_transformed


if __name__ == "__main__":
    frame_broadcaster = Broadcaster()
    frame_handler = FrameHandler()
    
    frame_broadcaster.broadcast_marker_tf(1, 1, 2, 3)

    print(frame_handler.convert_frame())
