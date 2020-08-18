#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import ropy as rp
import spatialmath as sm
import numpy as np
import time
import random

n = 9
tmax = 15
pQuad = rp.PandaURDF()
pQuad.name = 'Quad Panda'
pProj = rp.PandaURDF()
pProj.name = 'Proj Panda'
qlim = pQuad.qlim
rang = np.abs(qlim[0, :]) + np.abs(qlim[1, :])


pQuad.q = pQuad.qr
pProj.q = pProj.qr
pQuad.base = sm.SE3.Ty(0.4)
pProj.base = sm.SE3.Ty(-0.4)


def rand_q(k=0.4):
    q = np.zeros(n)
    for i in range(n):
        off = k * rang[i]
        q[i] = random.uniform(qlim[0, i] + off, qlim[1, i] - off)
    return q


def find_pose():
    q = rand_q()
    # pQuad.q = q
    # pProj.q = q
    return pQuad.fkine(q), pProj.fkine(q)


def check_limit(robot):
    limit = False
    off = 0.00
    for i in range(n):
        if robot.q[i] <= (qlim[0, i] + off) or robot.q[i] >= (qlim[1, i] - off):
            return True

    return limit


def mean(fail, m, fm, tot):
    fq = 0
    fp = 0
    mmq = 0.0
    mmp = 0.0
    mfmq = 0.0
    mfmp = 0.0
    j = 0

    for i in range(tot):
        if fail[i, 0]:
            fq += 1
        if fail[i, 1]:
            fp += 1

        if not fail[i, 0] and not fail[i, 1]:
            j += 1
            mmq += m[i, 0]
            mfmq += fm[i, 0]
            mmp += m[i, 1]
            mfmp += fm[i, 1]

    j = np.max([1, j])
    mmq = mmq/j
    mfmq = mfmq/j
    mmp = mmp/j
    mfmp = mfmp/j

    print("Quad: fails: {0}, mmean: {1}, mfinal: {2}".format(
        fq, np.round(mmq, 4), np.round(mfmq, 4)))

    print("Proj: fails: {0}, mmean: {1}, mfinal: {2}".format(
        fp, np.round(mmp, 4), np.round(mfmp, 4)))


env = rp.backend.Sim()
env.launch()


arrivedq = False
arrivedp = False

env.add(pQuad)
env.add(pProj)
time.sleep(1)

dt = 0.05

tests = 1000

m = np.zeros((tests, 2))
fm = np.zeros((tests, 2))
fail = np.zeros((tests, 2))

for i in range(tests):
    arrivedq = False
    arrivedp = False
    failq = False
    failp = False
    it = 0
    mq = 0
    mp = 0

    q_init = rand_q()

    pQuad.q = q_init.copy()
    pProj.q = q_init.copy()
    pQuad.qd = np.zeros(n)
    pProj.qd = np.zeros(n)
    env.step(dt)
    # time.sleep(2)

    Tq, Tp = find_pose()

    start = time.time()

    while (
            (not arrivedq or not arrivedp) and 
            (time.time() - start) < tmax and
            not failq and not failp):

        mq += pQuad.manipulability()
        mp += pProj.manipulability()

        if not arrivedq and not failq:
            vq, arrivedq = rp.p_servo(pQuad.fkine(), Tq, 1, threshold=0.25)
            pQuad.qd = np.linalg.pinv(pQuad.jacobe()) @ vq
        else:
            pQuad.qd = np.zeros(n)

        if not arrivedp and not failp:
            null = np.linalg.pinv(pProj.jacob0()) @ pProj.jacob0() @ pProj.jacobm()
            vp, arrivedp = rp.p_servo(pProj.fkine(), Tp, 1, threshold=0.25)
            pProj.qd = (np.linalg.pinv(pProj.jacobe()) @ vp) + null.flatten()
        else:
            pProj.qd = np.zeros(n)

        if check_limit(pQuad):
            failq = True

        if check_limit(pProj):
            failp = True

        env.step(dt * 1000)
        it += 1

    fail[i, 0] = not arrivedq
    fail[i, 1] = not arrivedp

    m[i, 0] = mq / it
    m[i, 1] = mp / it

    fm[i, 0] = pQuad.manipulability()
    fm[i, 1] = pProj.manipulability()

    print("Quad: {0}, mean: {1}, final: {2}".format(
        arrivedq, np.round(m[i, 0], 4), np.round(fm[i, 0], 4)))

    print("Proj: {0}, mean: {1}, final: {2}".format(
        arrivedp, np.round(m[i, 1], 4), np.round(fm[i, 1], 4)))
    mean(fail, m, fm, i+1)
    print()
