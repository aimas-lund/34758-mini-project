import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from coordinate_transformer import coordinateTransformer

class QRHandler:
    """
    Look for QR codes and decode/unpack them
    """
    _NUM_OF_QR_MARKERS = 5

    def __init__(self, world_x, world_y, 
                word=['']*_NUM_OF_QR_MARKERS, 
                qr_found=[False]*_NUM_OF_QR_MARKERS):
        # Code word found with QR codes. Structure: [String,String,String,String,String]
        self.word = word
        # List to keep track of which QR markers have been discovered. Structure: [Boolean,Boolean,Boolean,Boolean,Boolean]
        self.qr_found = qr_found
        # Robots position in the world frame:
        self.world_x = world_x
        self.world_y = world_y
        self.coordinate_transformer = coordinateTransformer()
        self.next_qr = Pose()

    def unpack_code_message(self,msg):
        """
        Input: message string
        Output: x, y, x_next, y_next, n, l (letter of word)

        msg is of the type "X=0.10\r\nY=3.50\r\nX_next=-3.1\r\nY_next=2.0\r\nN=3\r\nL=a"
        """
        msg_components = msg.split("\r\n")

        x = float(msg_components[0].split("=")[1])
        y = float(msg_components[1].split("=")[1])
        x_next = float(msg_components[2].split("=")[1])
        y_next = float(msg_components[3].split("=")[1])
        n = int(msg_components[4].split("=")[1])
        l = msg_components[5].split("=")[1]

        return x, y, x_next, y_next, n, l


    def code_message_cb(self, msg):
        """
        Logic for when a QR message is received (event)
        """

        #rospy.loginfo("Receieved code_message: " + str(msg.data))

        data = str(msg.data)
        if data != "" and self.world_x != 0.0:
            x, y, x_next, y_next, n, l = self.unpack_code_message(data)

            # If the found QR marker is the first that we have discovered, 
            # we need to set the translation and rotation between the frames
            if not any(self.qr_found):
                self.coordinate_transformer.calculate_translation_and_rotation(self.world_x, self.world_y, x, y)

            self.word[n-1] = l
            self.qr_found[n-1] = True

            rospy.logdebug(" -- qr_position_handler loop --")
            rospy.logdebug("Found QR number: " + str(n) + ", with letter: " + l)
            rospy.logdebug("please navigate to: " + str(x_next) + "," + str(y_next) + ", cur pos: " + str(self.world_x) + "," + str(self.world_y))

            self.next_qr = self.coordinate_transformer.hidden_cord_to_world_cord(x_next, y_next)        

    def object_position_cb(self,msg):
        """
        Keep track of QR position (event)
        Will be (0,0) if no QR has been found yet
        """
        #rospy.loginfo("Receieved object_position: " + str(msg.pose.position))
        self.world_x = msg.pose.position.x
        self.world_y = msg.pose.position.y


    def qr_reader(self):
        rospy.Subscriber("/visp_auto_tracker/object_position",
                        PoseStamped , self.object_position_cb)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                        String, self.code_message_cb)

    def print_word(self):
        return ''.join(self.word)
