#!/usr/bin/env python
"""
@author: Jesse Haviland
"""

# import numpy.testing as nt
# import numpy as np
import ropy as rp
# import spatialmath as sm
import unittest


class TestPyPlot(unittest.TestCase):

    def test_PyPlot(self):
        panda = rp.PandaMDH()
        env = rp.PyPlot()
        env.launch()
        env.add(panda)
        env.step()
        env._plot_handler(None, None)
        env.close()

    def test_unimplemented(self):
        # TODO remove these as implemented
        env = rp.PyPlot()
        env.reset()
        
        env.restart()
        env.remove()
