#! /usr/bin/env python3

'''
Simple movements for testing
'''

# imports 
import tf
import rospy
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from moveo_moveit.Moveo_class import Moveo
from moveo_moveit.msg import ArmJointState

# moveo model
robot = Moveo()
q0 = robot.qz
mask = [1,1,1,0,1,1]
home_tf = robot.fkine(robot.qz)
T = sm.SE3(0.3, 0.0, 0.4)
pick_T = sm.SE3(0.3, -0.34, 0.15)
drop_T = sm.SE3(0.3, 0.34, 0.35)
# R = sm.SE3.Rz(90,'deg')
# inverse kinematics
# T *= home_tf
ik = robot.ikine_LM(T, q0=q0, mask=mask)
# print(ik.q)
robot.plot(ik.q, block=True)
# robot.teach(ik.q, block=True)

# gear ratios:
'''
Joint 1: 10:1
Joint 2: 75:14
Joint 2: 75:14
Joint 3: 20:1
Joint 4: 1:1
Joint 5: 22:5
'''
# motors raw pulse/rev is 6400
# pulse/degree after gear reduction is: (6400 * GR) / 360 -> one degree rotation requires this 'n' pulse
pulse_per_deg = [((6400 * 10/1) / 360),# * -1,
                ((6400 * 75/14) / 360),# * -1, 
                # ((6400 * 75/14) / 360),
                ((6400 * 20/1) / 360),# * -1,
                ((6400 * 1/1) / 360),
                ((6400 * 22/5) / 360),# * -1,
                ]
# print(ik.q,"\n",robot.qz)
ik.q -= robot.qz
# compute joint angles
joint_angles = np.round(ik.q * (180/np.pi))                 # in angles
joint_positions = joint_angles * pulse_per_deg
joint_positions = np.append(joint_positions, 0)       # gripper value
joint_positions = joint_positions.astype(np.int16)
# print(joint_angles, "\n", joint_positions)

# positions
# home = [0, 0, 0, 0, 0, 90]  # upright position
# position1 = [-10000, -3000, -10000, 5000, 5000, 0]
# position2 = [-4600, -2400, -18410, 91, -800, 0]
# pos1 = [8000, 0, 0, 0, 0, 90]
# pos3 = [-8000, 0, 0, 0, 0, 90]
# pos2 = [0, 0, -8000, 0, 0, 90]
go_to = joint_positions.tolist()

def compute_joint_positions(val_ik, grip_val):
    joint_angles = np.round(val_ik * (180/np.pi))                 # in angles
    joint_positions = joint_angles * pulse_per_deg
    joint_positions = np.append(joint_positions, grip_val)       # gripper value
    joint_positions = joint_positions.astype(np.int16)
    print(joint_angles, "\n", joint_positions)
    return joint_positions.tolist()

# goal generator function
def get_goal(points):
    goal = ArmJointState()
    goal.position1 = points[0]
    goal.position2 = points[1]
    goal.position3 = points[2]
    goal.position4 = points[3]
    goal.position5 = points[4]
    goal.position6 = points[5]
    return goal

# wait functionn unitl the robot reaches the arg transform (using "/tf")
def waitToGoal(target_pose):
    # distance function
    def distance(diff_x, diff_y, diff_z):
        return np.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
    
    tx, ty, tz = target_pose.t
    tolerance = 0.05                # in meters
    listener = tf.TransformListener()
    while True:
        try:
            trans,rot = listener.lookupTransform("base","ee",rospy.Time(0))
            cx, cy, cz = trans     # robot's current x, y, z
            if (distance(tx-cx, ty-cy, tz-cz) <= tolerance):
                break
            print(distance(tx-cx, ty-cy, tz-cz))
        except:
            rospy.sleep(0.5)
            continue 
    print(f"Reached target: {target_pose.t}")


def main():
    pub = rospy.Publisher('joint_steps', ArmJointState,queue_size=4)
    rospy.init_node("movements",anonymous=True)
    rate = rospy.Rate(1) # 0.1 20hz

    while not rospy.is_shutdown():
        # move to pick pose
        '''pick_ik = robot.ikine_LM(pick_T, q0=q0, mask=mask)
        jt_position = compute_joint_positions(pick_ik.q - robot.qz, 0)
        goal_msg = get_goal(jt_position)
        pub.publish(goal_msg)
        waitToGoal(pick_T)
        rospy.sleep(2)
        
        # close gripper
        jt_position = compute_joint_positions(pick_ik.q - robot.qz, 179)
        goal_msg = get_goal(jt_position)
        pub.publish(goal_msg)
        rospy.sleep(5)

        drop_ik = robot.ikine_LM(drop_T, q0=q0, mask=mask)
        jt_position = compute_joint_positions(drop_ik.q - robot.qz, 179)
        goal_msg = get_goal(jt_position)
        pub.publish(goal_msg)   
        waitToGoal(drop_T)
        rospy.sleep(2)

        jt_position = compute_joint_positions(drop_ik.q - robot.qz, 0)
        goal_msg = get_goal(jt_position)
        pub.publish(goal_msg)
        rospy.sleep(5)
        
        home_ik = robot.ikine_LM(home_tf, q0=q0, mask=mask)
        jt_position = compute_joint_positions(home_ik.q - robot.qz, 90)
        goal_msg = get_goal(jt_position)
        pub.publish(goal_msg)   
        waitToGoal(home_tf)
        rospy.sleep(20)
        
        break'''

        goal_msg = get_goal(go_to)
        pub.publish(goal_msg)
        # waitToGoal(T)
        rospy.sleep(10)

        # move to position
        # goal_msg = get_goal(go_to)
        # goal_msg = get_goal(pos1)
        # pub.publish(goal_msg)
        # rospy.sleep(2)

        # goal_msg = get_goal(home)
        # pub.publish(goal_msg)
        # rospy.sleep(2)
# 
        # goal_msg = get_goal(pos3)
        # pub.publish(goal_msg)
        # rospy.sleep(2)
# 
        # goal_msg = get_goal(home)
        # pub.publish(goal_msg)
        # rospy.sleep(2)
# 
        # goal_msg = get_goal(pos2)
        # pub.publish(goal_msg)
        # rospy.sleep(2)
# 
        # goal_msg = get_goal(home)
        # pub.publish(goal_msg)
        # rospy.sleep(2)

        # return to home:
        # goal_msg = get_goal(home)
        # pub.publish(goal_msg)
        # rospy.sleep(10)


if __name__ == '__main__':
    main()
    rospy.spin()

    # try:
        # publish_detected_object()
    # except rospy.ROSInterruptException:
        # pass