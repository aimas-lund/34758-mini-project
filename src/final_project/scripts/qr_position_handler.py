import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose

import final_util
import math
import numpy as np

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

        self.qr_robot_diff = None
        self.robot_pose = None

        self.stop_wandering = False

        self.next_qr_pose = Pose()

        self.QRcodes = ['']*self._NUM_OF_QR_MARKERS
        self.QRcodesfound = ['']*self._NUM_OF_QR_MARKERS

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
            print(hidden_x, hidden_y, hidden_x_next, hidden_y_next, n, l )

            if l not in self.QRcodesfound:
                self.word[n-1] = l
                self.qr_found[n-1] = True

                qr = QR(hidden_x,hidden_y,hidden_x_next,hidden_y_next,n,l)
                self.QRcodes[n-1]=qr
                self.QRcodesfound[n-1]=l

            #current_qr_pose = final_util.calculate_real_qr_pose(self.robot_pose, self.qr_robot_diff)

            #   self.next_qr_pose = final_util.calculate_next_qr_pose(self.robot_pose, self.qr_robot_diff, hidden_x, hidden_y)

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

    def calculate_QR_XY(self,x1, y1):

        x=x1*self.ctheta-y1*math.sqrt(1-math.pow(self.ctheta,2))+self.X
        y=x1*math.sqrt(1-math.pow(self.ctheta,2))+y1*self.ctheta+self.Y

        return x, y
    
    def calculate_destination(self,next_x, next_y):
        beta=math.atan2(next_y, next_x)
        if beta<0:
            beta=beta+2*math.pi
        #rot=beta-alpha
        if beta<=10*math.pi/180 or beta>=350*math.pi/180: 
            new_x=next_x-1.5
            new_y=next_y
            rot=0
        if beta>10*math.pi/180 and beta<170*math.pi/180 : 
            new_x=next_x
            new_y=next_y-1.5
            rot=3.14/2
        if beta>=170*math.pi/180 and beta<190*math.pi/180 : 
            new_x=next_x+1.5
            new_y=next_y
            rot=3.14
        if beta>=190*math.pi/180 and beta<350*math.pi/180 : 
            new_x=next_x
            new_y=next_y+1.5
            rot=3*3.14/2
        return new_x, new_y, rot

    def map_qr_transformation(self):
        qrs= self.QRcodes
        print(qrs)
        indices = np.where(self.qr_found)[0]
        x1, y1, xn1, yn1 = qrs[indices[0]].getvalues()
        x2, y2, xn2, yn2 = qrs[indices[1]].getvalues()
        num=(x1-x2)*(xn1-xn2)+(y1-y2)*(yn1-yn2)
        det=(xn1-xn2)*(xn1-xn2)+(yn1-yn2)*(yn1-yn2)
        self.ctheta=num/det

        self.X=x1-xn1*self.ctheta+yn1*math.sqrt(1-math.pow(self.ctheta,2))
        self.Y=y1-xn1*math.sqrt(1-math.pow(self.ctheta,2))-yn1*self.ctheta