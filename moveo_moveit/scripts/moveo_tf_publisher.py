'''
    This is a python script for converting the joint_positions (initially as pulse values) to joint angles (in degrees).
    The script publishes the transform -> pose of the end effector.
'''

# imports
import tf
import rospy
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb
from moveo_moveit.Moveo_class import Moveo
from moveo_moveit.msg import ArmJointState
import tf.transformations

# declarations
pulse_per_deg = np.array([((6400 * 10/1) / 360) * -1,
                ((6400 * 75/14) / 360) * -1, 
                # ((6400 * 75/14) / 360),
                ((6400 * 20/1) / 360) * -1,
                ((6400 * 1/1) / 360),
                ((6400 * 22/5) / 360) * -1,
                ])
# moveo model
robot = Moveo()

# publish tf
def tf_pub_func(msg):
    br = tf.TransformBroadcaster()
    joint_positions = np.array([msg.position1, msg.position2, msg.position3, msg.position4, msg.position5]) 
    joint_angles = joint_positions / pulse_per_deg
    joint_angles_rad = joint_angles * (np.pi/180)
    ee_pose = robot.fkine(joint_angles_rad + robot.qz)
    trans = np.round(ee_pose.t, 4)
    matrix = np.eye(4,4); matrix[0:3, 0:3] = ee_pose.R
    q_rot = (tf.transformations.quaternion_from_matrix(matrix)) # quaternion
    br.sendTransform(translation=trans,
                     rotation=q_rot,
                     time=rospy.Time.now(),
                     child="ee",        # moveo end-effector
                     parent="base"      # moveo base-link
                    )


if __name__ == '__main__':
    rospy.init_node("EE_tf_publisher_node")
    sub = rospy.Subscriber("/joint_positions", ArmJointState, callback=tf_pub_func)
    rospy.spin()
