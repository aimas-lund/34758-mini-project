import rospy
import os
from geometry_msgs.msg import PoseStamped, Pose

class DebugPublishHandler:

    def __init__(self):
        self.publisher_map = {}


    def add_publish_channel(self, pub_name, publisher):
        if type(publisher) is not rospy.topics.Publisher:
            rospy.logerr("Publisher was not of type 'rospy.topics.Publisher'")
            return

        self.publisher_map.update({pub_name: publisher})


    def remove_publish_channel(self, pub_name):
        try:
            del self.publisher_map[pub_name]
        except KeyError:
            rospy.logerr("Debug-publisher '%s' was not instantiated in %s!", pub_name, os.path.basename(__file__))


    def publish(self, pub_name, value):
        self.publisher_map[pub_name].publish(value)

        return