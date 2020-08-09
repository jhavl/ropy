#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import numpy as np
import ropy as rp
import spatialmath as sm
from ropy.backend.PyPlot.RobotPlot import RobotPlot


class RobotPlot2(RobotPlot):

    def __init__(
            self, robot, ax, readonly, display=True,
            eeframe=True, name=True):

        super(RobotPlot2, self).__init__(
            robot, ax, readonly, display=display,
            jointaxes=False, shadow=False, eeframe=eeframe, name=name
        )
