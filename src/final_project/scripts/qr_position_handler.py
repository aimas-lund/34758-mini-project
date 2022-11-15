import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from debug_handler import DebugPublishHandler
import tf
import tf_conversions
import numpy as np
import math

import final_util

class QRHandler:
    """
    Look for QR codes and decode/unpack them
    """
    # TODO: make this dynamic, it can change
    number_of_qr_markers = 5
    _TAG = "QR POSITION HANDLER"

    def __init__(self, 
                listener,
                nav,
                word=['']*number_of_qr_markers, 
                qr_found=[False]*number_of_qr_markers,
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
        self.qr_robot_diff = None
        self.qr_robot_cov = 10
        self.next_qr_pose = Pose()

        self.qr_real = [None]*self.number_of_qr_markers # list of [x,y]
        self.qr_hidden = [None]*self.number_of_qr_markers # list of [x,y]

        self.listener = listener
        self.navigator = nav

        # hidden to real transformations
        self.rotation = None
        self.translation = None


    def calculateTranslationAndRotation(self): 
        """
            Function for computing translation and rotation. 
            Requires that minimum two QR markers have been discoved.
        """
        # found qr markers
        indices = [i for i in range(self.number_of_qr_markers) if self.qr_real[i]]

        realQR1 = self.qr_real[indices[0]]
        realQR2 = self.qr_real[indices[1]]
        hiddenQR1 = self.qr_hidden[indices[0]]
        hiddenQR2 = self.qr_hidden[indices[1]]

        # translate QR hidden/real positions into vectors
        realQRVector = np.subtract(realQR1, realQR2)
        hiddenQRVector = np.subtract(hiddenQR1, hiddenQR2)

        # compute angle between vectors
        real_length = np.linalg.norm(realQRVector)
        hidden_length = np.linalg.norm(hiddenQRVector)
        diff_length = np.linalg.norm(realQRVector - hiddenQRVector)
        theta = math.acos((math.pow(real_length, 2) + math.pow(hidden_length, 2) - math.pow(diff_length, 2)) / (2 * real_length * hidden_length))

        # compute translation and rotation
        self.rotation = np.array(((np.cos(theta), -np.sin(theta)), 
                                  (np.sin(theta),  np.cos(theta))))
        self.translation = np.subtract(realQR1, self.rotation.dot(hiddenQR1)).tolist()

        # log results
        rospy.logdebug("{}: Obtained rotation matrix -> {}".format(self._TAG, self.rotation))
        rospy.logdebug("{}: Obtained translation matrix -> {}".format(self._TAG, self.translation))


    def calculate_real(self, i):
        # rospy.logdebug(self._TAG + ": Calculating real position for qr_marker " + str(i) + " from hidden: " + str(self.qr_hidden[i]))
        real_transform = np.dot(self.rotation, self.qr_hidden[i]) + self.translation
        self.qr_real[i] = real_transform.tolist()
        # rospy.logdebug(self._TAG + ": Real position for qr_marker " + str(i) + " is: " + str(self.qr_real[i]))
        # rospy.logdebug(self._TAG + ": qr_real array: " + str(self.qr_real))
        

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

        # print message
        #rospy.loginfo("Receieved code_message: " + str(msg.data))

        # ensure both code_message and object_position have been received
        data = str(msg.data)
        if (data == ""):
            return
        if (self.qr_robot_diff == None):
            return
        if (self.qr_robot_diff.position.x == 0.0):
            return


        # decode message
        hidden_x, hidden_y, hidden_x_next, hidden_y_next, n, l = self.unpack_code_message(data)

        rospy.logdebug_throttle(0.5, self._TAG + ": Seeing qr_marker " + str(n-1) + ", covariance: " + str(self.qr_robot_cov))
    
        if self.word[n-1] == "":
            # process QR letter and hidden info, even if covariance is high (as cov only matters for position)
            self.word[n-1] = l
            # save hidden
            self.qr_hidden[n-1] = [hidden_x, hidden_y]
            self.qr_hidden[n % self.number_of_qr_markers] = [hidden_x_next, hidden_y_next]
            # rospy.logdebug(self._TAG + ": updated qr_hidden: " + str(self.qr_hidden))
            # rospy.logdebug(self._TAG + ": updated word: " + str(self.word))

        # don't calculate world position if covariance is too high
        if self.qr_robot_cov > 1e-5:
            return

        # don't calculate real pos if it has already been calculated
        if self.qr_real[n-1] != None:
            return

        rospy.logdebug(self._TAG + ": calculate real position of qr_marker " + str(n-1))
        # calculate the world position for current qr
        self.qr_real[n-1] = final_util.calculate_real_qr_xy(self.navigator.get_coordinates(), self.qr_robot_diff, self.listener)
        if self.debug_mode:
            self.debug_handler.publish('qr_pos', self.qr_real[n-1])

        # rospy.logdebug(self._TAG + ": qr_marker " + str(n-1) + " real is: " + str(self.qr_real[n-1]) + ", and hidden is: " + str(self.qr_hidden[n-1]))
        # rospy.logdebug(self._TAG + ": qr_real array: " + str(self.qr_real))

        # set found to true, all the way at the end to avoid concurrency problems
            

    def object_position_cb(self,msg):
        """
        Keep track of QR position (event)
        Will be (0,0) if no QR has been found yet
        """
        #rospy.loginfo("Receieved object_position: " + str(msg.pose))
        # covariance for x,x and y,y
        self.qr_robot_cov = msg.pose.covariance[0] + msg.pose.covariance[7]
        #rospy.logdebug("Covariance: " + str(self.qr_robot_cov))
        self.qr_robot_diff = msg.pose.pose


    def qr_reader(self):
        rospy.Subscriber("/visp_auto_tracker/object_position_covariance",
                        PoseWithCovarianceStamped , self.object_position_cb)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                        String, self.code_message_cb)

    def print_word(self):
        return ''.join(self.word)
