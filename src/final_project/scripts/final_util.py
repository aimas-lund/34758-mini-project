import rospy
from tf.transformations import quaternion_matrix, concatenate_matrices, translation_matrix


def calculate_real_qr_xy(qr_robot_diff, listener):
    """
    returns world position of QR
    """

    robot_diff_pose = [qr_robot_diff.position.x, qr_robot_diff.position.y, qr_robot_diff.position.z, 1]
    try:
        # lookupTransform between robot and camera
        (translation,rotation) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
        # translate the position
        robot_to_qr = concatenate_matrices(translation_matrix(translation),quaternion_matrix(rotation))
        qr_real_position = robot_to_qr.dot(robot_diff_pose)

        return [qr_real_position[0], qr_real_position[1]]
    except:
        print("Error calculation the real position of QR marker from camera")

    return None
