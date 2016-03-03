import unittest

from brian2 import *


class TestPoissonSpikeGeneration(unittest.TestCase):

    def test_simple_poisson_generator(self):
        P = PoissonGroup(10, 5*Hz)
        M = SpikeMonitor(P)
        run(10*ms)
        n = M.t
        # plot(M.t/ms, M.v[0], '-b', lw=2, label='N0: membrane potential')
        # xlabel('Time (ms)')
        # ylabel('t')
        # legend(loc='best')
        # show()

if __name__ == '__main__':
    unittest.main()