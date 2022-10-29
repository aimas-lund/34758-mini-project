import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from broadcast_marker_tf import broadcast_marker_tf

# Initialize global variables for the discovered word and
# a list to keep track of which qr markers we have discovered already
global word, qr_found
word = ["", "", "", "", ""]
qr_found = [False, False, False, False, False]

# Robots position in the world frame
global world_x, world_y


def unpack_code_message(msg):
    # msg is of the type "X=0.10\r\nY=3.50\r\nX_next=-3.1\r\nY_next=2.0\r\nN=3\r\nL=a"

    msg_components = msg.split("\r\n")

    x = float(msg_components[0].split("=")[1])
    y = float(msg_components[1].split("=")[1])
    x_next = float(msg_components[2].split("=")[1])
    y_next = float(msg_components[3].split("=")[1])
    n = int(msg_components[4].split("=")[1])
    l = msg_components[5].split("=")[1]

    print("QR code detected:", msg_components)

    return x, y, x_next, y_next, n, l


def code_message_cb(msg):
    if msg != "":
        x, y, x_next, y_next, n, l = unpack_code_message(msg)

        # If the found QR marker is the first that we have discovered, we need to broadcast the hidden frame
        if not any(qr_found):
            broadcast_marker_tf(world_x, world_y, x, y)

        word[n-1] = l
        qr_found[n-1] = True

        # Here we should call a navigation script...
        print("please navigate to", x_next, y_next)


def object_position_cb(msg):
    print(msg)

    # TODO: Not sure if this is the correct way to decompose msg
    world_x = msg.position.x
    world_y = msg.position.y


def qr_reader():
    rospy.Subscriber("/visp_auto_tracker/code_message",
                     String, code_message_cb)

    # Not sure if it exactly is a Pose we get back from object_position?
    rospy.Subscriber("visp_auto_tracker/object_position",
                     Pose, object_position_cb)
