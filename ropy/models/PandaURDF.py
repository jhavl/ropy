#!/usr/bin/env python

import numpy as np
import os
from ropy.robot.ETS import ETS


class PandaURDF(ETS):

    def __init__(self):

        fpath = 'ropy/models/xarco/panda/robots/panda_arm_hand.urdf.xacro'
        abspath = os.getcwd() + '/ropy/models/xarco/panda/robots/'

        args = super(PandaURDF, self).urdf_to_ets_args(fpath)

        super(PandaURDF, self).__init__(
            args[0],
            name=args[1])

        self.manufacturer = 'Franka Emika'
        # self.ee_link = self.ets[6]

        self._qz = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        self._qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi/4, 0, 0])

        for link in self.ets:
            for gi in link.geometry:
                if gi.filename[0] != '/':
                    gi.filename = abspath + gi.filename
            # print(link.name)
            # print(link.geometry)

    @property
    def qz(self):
        return self._qz

    @property
    def qr(self):
        return self._qr
