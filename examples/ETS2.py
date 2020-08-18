#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import ropy as rp
import spatialmath as sm
import numpy as np
import time





env = rp.backend.Sim()
env.launch()

pQuad = rp.PandaURDF()
pProj = rp.PandaURDF()
pQuad.q = pQuad.qr
pProj.q = pQuad.qr
pQuad.base = sm.SE3.Ty(0.4)
pProj.base = sm.SE3.Ty(-0.4)

Tep = pQuad.fkine() * sm.SE3.Tx(-0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.4) * sm.SE3.Rx(0.6)* sm.SE3.Ry(0.6)
Tep2 = pProj.fkine() * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(-0.4) * sm.SE3.Rx(0.6)* sm.SE3.Ry(0.6)

arrived = False
env.add(pQuad)
env.add(pProj)
time.sleep(1)

dt = 0.05

while not arrived:

    start = time.time()
    v, arrived = rp.p_servo(pQuad.fkine(), Tep, 1)
    v2, _ = rp.p_servo(pProj.fkine(), Tep2, 1)
    pQuad.qd = np.linalg.pinv(pQuad.jacobe()) @ v
    pProj.qd = np.linalg.pinv(pProj.jacobe()) @ v
    env.step(dt * 1000)
    stop = time.time()

    # if stop - start < dt:
        # time.sleep(dt - (stop - start))