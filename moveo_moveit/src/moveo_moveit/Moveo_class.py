#!/usr/bin/env python
'''
    Description: the file moveo.py contains Moveo Class, which is a custom robot model for the open sourced robotic arm BCN3D Moveo
'''

# NOTE
# theta is not used, I don't know why?
# d and a are in metres
# angles are in rad
# (WIP) -> limit qlim is not updated for moveo (based on urdf file)


# imports
import numpy as np
from math import pi
import spatialmath as sm
import roboticstoolbox as rtb
from spatialmath.base import trotz, transl
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, RevoluteMDH, PrismaticMDH

# Moveo Robotic Arm model class
class Moveo(DHRobot):
    """
    A class representing the BCN3D Moveo robot arm.

    ``Moveo()`` is a class which models a open sourced robot and
    describes its kinematic characteristics using modified DH
    conventions.

    .. note::
        - SI units of metres are used.
        - The model includes a tool offset. (WIP)

    :references:
        - https://repositorio-aberto.up.pt/bitstream/10216/113577/2/276343.pdf

    .. code author:: Logesh G
    """

    def __init__(self):
        
        # Denavit-Hartenberg parameters
        L = [
            RevoluteDH(
                #offset=0.0,
                d=0.232,
                a=0.0,
                alpha=-pi/2,
                qlim=np.array([-pi/2, pi/2]),
                
                # r=[4.02e-05, 0.0906, 0.000102],
                # m=4.2526,
                # I=[
                #     0.00341,
                #     0.0186,
                #     0.022,
                #     8.9e-06,
                #     1.18e-07,
                #     -3.65e-06,
                # ],
                # G=1,
            ),
            RevoluteDH(
                #offset=0.0,
                d=0.0,
                a=0.221,
                alpha=0.0,
                qlim=np.array([-pi, 0.0]),
                
                # m=1.93,
                # I=[
                    # 0.0121, 
                    # 0.0114, 
                    # 0.000703, 
                    # -3.56e-06, 
                    # -9.06e-06, 
                    # 0.000143,
                # ],
                # G=1,
            ),
            RevoluteDH(
                #offset=pi/2,
                d=0.0,
                a=0.0,
                alpha=pi/2,
                qlim=np.array([0.0, pi]),
                
                # m=1.14,
                # I=[
                    # 0.000236, 
                    # 0.000291, 
                    # 0.000525, 
                    # -3.84e-06, 
                    # 4.13e-06, 
                    # -1.04e-07,
                # ],
                # G=1,
            ),
            RevoluteDH(
                #offset=0.0,
                d=0.223, 
                a=0.0, 
                alpha=-pi/2,
                qlim=np.array([-pi/2, pi/2]),
                
                # m=0.63,
                # I=[
                    # 0.000172, 
                    # 0.000105, 
                    # 7.89e-05, 
                    # 1.34e-06, 
                    # 8.52e-06, 
                    # 5.14e-05,
                # ],
                # G=1,
            ),
            RevoluteDH(
                #offset=-pi/2,
                d=0.0,
                a=0.099,
                alpha=0.0,
                qlim=np.array([-pi, 0.0]),
                
                # m=0.199,
                # I=[
                    # 6.27e-05, 
                    # 9.78e-05, 
                    # 9.58e-05, 
                    # 4.26e-06, 
                    # 4.02e-05, 
                    # 8.59e-07,
                # ],
                # G=1,
            ),
        ]

        super().__init__(
            L,
            name="Moveo",
            manufacturer="BCN3D",
            # meshdir="/home/logesh/catkin_ws/src/moveo_ros/moveo_urdf/meshes",
        )

        # zero angles position
        self.qz = np.array([0, -pi/2, pi/2, 0, -pi/2]) # [0, 0, pi/2, 0, -pi/2]  



if __name__ == "__main__":  # pragma nocover

    moveo = Moveo()
    print(moveo)
    # moveo.plot(moveo.qz,block=True)
    moveo.teach(moveo.qz, block=True)
