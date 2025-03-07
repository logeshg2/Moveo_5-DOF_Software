import roboticstoolbox as rtb
import spatialmath as sm
from Moveo import Moveo
import numpy as np
from math import pi

# moveo model
robot = Moveo()

T = sm.SE3.Tx(0.3)
T *= sm.SE3.Tz(0.4)
# R = sm.SE3.Rz(90,'deg')
# print(T)

# inverse kinematics
ik = robot.ikine_LMS(T) # * R)
joint_angles = np.round(ik.q) * (180/pi)
#print(joint_angles)
#robot.teach(robot.qz, block=True)

# pulse/angle
'''

'''
pulse_per_deg = [((6400 * 10/1) / 360),
                ((6400 * 75/14) / 360),
                ((6400 * 150/7) / 360),
                ((6400 * 1/1) / 360),
                ((6400 * 22/5) / 360),
                ]
#print(joint_angles * pulse_per_deg)

# forward kinematics
fk = robot.fkine(ik.q)

# plot the robot
robot.plot(ik.q, block=True)
print(joint_angles)
# teach window for robot
# robot.teach(ik.q, block=True)

