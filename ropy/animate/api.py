import zerorpc
import ropy as rp
import numpy as np


panda = rp.PandaMDH()
panda.q = panda.qr

q = [0, -0.3, 0, -2.2, 0, 2.0, np.pi/4]
# q = [0, 0, 0, -1, 0, 0, 0]
# q = panda.q.tolist()

l = []
for i in range(panda.n):
    # m = min(0, i-1)
    # l.append(panda.A([m, i]).A.tolist())
    # lib = panda.A(i).A
    # lie = panda.links[i].A(panda.q[i]).A
    # l.append(lib.tolist())
    # l.append(lie.tolist())
    li = [panda.links[i].sigma, panda.links[i].mdh, panda.links[i].theta, panda.links[i].d, panda.links[i].a, panda.links[i].alpha]
    l.append(li)


l[3][3] = 0.1

# l1 = panda.A([0, 1])



sim = zerorpc.Client()
sim.connect("tcp://127.0.0.1:4242")

ob = ["SerialLinkMDH", l]
id = sim.robot(ob)

q_ob = [id, q]
sim.q(q_ob)

# qd = [0, 0, 0, 0.1, 0, 0, 0]
# qd_ob = [id, qd]
# sim.qd(qd_ob)


# v = np.array([[0.01, 0.01, 0.01, 0, 0, 0]]).T

# for i in range(5000):
#     panda.q = sim.get_q(id)
#     qd = np.linalg.pinv(panda.jacobe()) @ v
#     qd = qd.tolist()
#     sim.qd([id, qd])

#     sim.step(1)



