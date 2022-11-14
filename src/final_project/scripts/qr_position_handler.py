
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
import final_util
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, concatenate_matrices, translation_matrix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
import tf

class QR:
    def __init__(self,x,y,x_next,y_next,n,l):
        self.x = x
        self.y = y
        self.x_next = x_next
        self.y_next = y_next
        self.n = n
        self.l = l
    def getvalues(self):
        return self.x,self.y,self.x_next,self.y_next


class QRHandler:
    """
    Look for QR codes and decode/unpack them
    """
    _NUM_OF_QR_MARKERS = 5

    def __init__(self, 
                word=['']*_NUM_OF_QR_MARKERS, 
                qr_found=[False]*_NUM_OF_QR_MARKERS):
        # Code word found with QR codes. Structure: [String,String,String,String,String]
        self.word = word
        # List to keep track of which QR markers have been discovered. Structure: [Boolean,Boolean,Boolean,Boolean,Boolean]
        self.qr_found = qr_found
        #self.qr_real_pos = None
        self.robot_pose = None
        self.stop_wandering = False
        self.next_qr_pose = Pose()
        self.QRcodes = ['']*self._NUM_OF_QR_MARKERS
        self.QRcodesfound = ['']*self._NUM_OF_QR_MARKERS

        self.qr_robot_diff = None

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
        if data != "":
            hidden_x, hidden_y, hidden_x_next, hidden_y_next, n, l = self.unpack_code_message(data)
            
            # I don't think this check is necessary!:
            #if self.qr_real_pos.position.x != 0.0:
            
            self.new_qrfound = False
            print(hidden_x, hidden_y, hidden_x_next, hidden_y_next, n, l )
            if l not in self.QRcodesfound:
                self.word[n-1] = l
                self.qr_found[n-1] = True
                qr = QR(hidden_x,hidden_y,hidden_x_next,hidden_y_next,n,l)
                self.QRcodes[n-1]=qr
                self.QRcodesfound[n-1]=l

                self.new_qrfound = True            

            # TODO: calculate the real to hidden translation system
             
    def object_position_cb(self,msg):
        """
        Keep track of QR position (event)
        Will be (0,0) if no QR has been found yet
        """
        #rospy.loginfo("Receieved object_position: " + str(msg.pose))
        self.qr_robot_diff = msg.pose
        

    def qr_reader(self):
        rospy.Subscriber("/visp_auto_tracker/object_position_covariance",
                        PoseWithCovarianceStamped , self.object_position_cb)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                        String, self.code_message_cb)
    def print_word(self):
        return ''.join(self.word)

    def update_robot_pose(self, robot_pose):
        self.robot_pose = robot_pose

    def hidden2real(self,hidden):
        self.rotation
        self.translation
        self.real = self.rotation.dot(hidden)+self.translation
        return self.real

    def odom_callbak(self,msg):
        pose = msg.pose.pose        
        self.odo_pose = [(pose.position.x,pose.position.y,pose.position.z),(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)]

        
    def transInit(self,realPos1, realPos2, hiddenPos1, hiddenPos2): #give args as numpy arrays
        A = np.array([[hiddenPos1[0], -hiddenPos1[1], 1 ,0],
                    [hiddenPos1[1], hiddenPos1[0], 0 ,1],
                    [hiddenPos2[0], -hiddenPos2[1], 1 ,0],
                    [hiddenPos2[1], hiddenPos2[0], 0 ,1]])
        b = np.array([realPos1[0], realPos1[1], realPos2[0], realPos2[1]])
        x = np.dot(np.linalg.inv(A),b)
        c = x[0]
        s = x[1]
        xT = x[2]
        yT = x[3]
        self.rotation = np.array(((c, -s), (s, c)))
        self.translation = np.array([xT,yT])


    def qr_status_callback(self,msg):
        self.status_qr = msg.data
    
    def qr_msg_callback(self,msg):
        self.msg_qr = msg.data

    def qr_tf_cov_callback(self,msg):
        self.ob_postion_relative = msg.pose