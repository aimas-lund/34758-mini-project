import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped 
from broadcast_marker_tf import broadcaster

class QRkanker:

    def __init__(self, word, qr_found, world_x, world_y):
        # Code word found with QR codes. Structure: [String,String,String,String,String]
        self.word = word
        # List to keep track of which QR markers have been discovered. Structure: [Boolean,Boolean,Boolean,Boolean,Boolean]
        self.qr_found = qr_found
        # Robots position in the world frame:
        self.world_x = world_x
        self.world_y = world_y
        self.broadcaster = broadcaster()

    def unpack_code_message(self,msg):
        # msg is of the type "X=0.10\r\nY=3.50\r\nX_next=-3.1\r\nY_next=2.0\r\nN=3\r\nL=a"
        print(msg)
        msg_components = msg.split("\r\n")

        x = float(msg_components[0].split("=")[1])
        y = float(msg_components[1].split("=")[1])
        x_next = float(msg_components[2].split("=")[1])
        y_next = float(msg_components[3].split("=")[1])
        n = int(msg_components[4].split("=")[1])
        l = msg_components[5].split("=")[1]

        return x, y, x_next, y_next, n, l


    def code_message_cb(self,msg):
        data = str(msg.data)
        if data != "":
            x, y, x_next, y_next, n, l = self.unpack_code_message(data)

            # If the found QR marker is the first that we have discovered, we need to broadcast the hidden frame
            if not any(self.qr_found):
                self.broadcaster.broadcast_marker_tf(self.world_x, self.world_y, x, y)
                # TODO: set wander.move to False

            self.word[n-1] = l
            self.qr_found[n-1] = True

            # Here we should call a navigation script...
            print("please navigate to", x_next, y_next)


    def object_position_cb(self,msg):

        # TODO: Not sure if this is the correct way to decompose msg
        self.world_x = msg.pose.position.x
        self.world_y = msg.pose.position.y


    def qr_reader(self):
        # Not sure if it exactly is a Pose we get back from object_position?
        rospy.Subscriber("/visp_auto_tracker/object_position",
                        PoseStamped , self.object_position_cb)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                        String, self.code_message_cb)


