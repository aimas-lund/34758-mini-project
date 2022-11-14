import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from debug_handler import DebugPublishHandler

import final_util

class QRHandler:
    """
    Look for QR codes and decode/unpack them
    """
    _NUM_OF_QR_MARKERS = 5

    def __init__(self, 
                word=['']*_NUM_OF_QR_MARKERS, 
                qr_found=[False]*_NUM_OF_QR_MARKERS,
                debug=False):
        # a map of publishers which the QR Handler can publish intermediate values with 
        self.debug_mode = debug
        if debug:
            self.debug_handler = DebugPublishHandler()
            self.debug_handler.add_publish_channel('qr_pos', rospy.Publisher('qr_pos', Pose, queue_size=1))
        else:
            self.debug_handler = None
        
        # Code word found with QR codes. Structure: [String,String,String,String,String]
        self.word = word
        # List to keep track of which QR markers have been discovered. Structure: [Boolean,Boolean,Boolean,Boolean,Boolean]
        self.qr_found = qr_found
        self.qr_robot_diff = None
        self.robot_pose = None
        self.next_qr_pose = Pose()


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
        # ensure both code_message and object_position have been received
        if data != "" and self.qr_robot_diff.position.x != 0.0:
            hidden_x, hidden_y, hidden_x_next, hidden_y_next, n, l = self.unpack_code_message(data)

            self.word[n-1] = l
            self.qr_found[n-1] = True

            current_qr_pose = final_util.calculate_real_qr_pose(self.robot_pose, self.qr_robot_diff)
            if self.debug_mode:
                self.debug_handler.publish('qr_pos', current_qr_pose)

            self.next_qr_pose = final_util.calculate_next_qr_pose(self.robot_pose, self.qr_robot_diff, hidden_x, hidden_y, hidden_x_next, hidden_y_next)
            rospy.logdebug(" -- qr_position_handler loop --")
            rospy.logdebug("Found QR number: " + str(n) + ", with letter: " + l + ", at real position: " + str(current_qr_pose))
            rospy.logdebug("navigate to: " + str(self.next_qr_pose.position) + ", from current pos: " + str(self.robot_pose.position))

            # TODO: calculate the real to hidden translation system
             

    def object_position_cb(self,msg):
        """
        Keep track of QR position (event)
        Will be (0,0) if no QR has been found yet
        """
        #rospy.loginfo("Receieved object_position: " + str(msg.pose))
        self.qr_robot_diff = msg.pose


    def qr_reader(self):
        rospy.Subscriber("/visp_auto_tracker/object_position",
                        PoseStamped , self.object_position_cb)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                        String, self.code_message_cb)

    def print_word(self):
        return ''.join(self.word)

    def update_robot_pose(self, robot_pose):
        self.robot_pose = robot_pose
