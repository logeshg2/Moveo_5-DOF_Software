import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from Moveo import Moveo

env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
# panda = Panda()
panda.q = panda.qr

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.0, 0.0, 0.6)
# Tep = panda.fkine(panda.q) * sm.SE3.RPY(0.0, 0.0, 90.0, unit="deg")
# print(panda.fkine(panda.q), sm.SE3.Trans(0.2, 0.2, 0.45), sm.SE3.RPY(0.0, 0.0, 90.0, unit="deg"))
# print(panda.fkine(panda.q), sm.SE3.Trans(0.0, 0.0, 0.6))
# Tep = sm.SE3.Trans(0.0, 0.0, 0.6)
# print(panda.ikine_GN(Tep))
print(Tep)

arrived = False
env.add(panda)

dt = 0.05

while not arrived:
    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, 1)
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v
    env.step(dt)

# Uncomment to stop the browser tab from closing
# env.hold()
