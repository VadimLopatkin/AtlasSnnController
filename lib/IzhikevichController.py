from brian2 import *
import numpy as np

class IzhikevichController():
    def __init__(self, number_of_neurons):
        eqs = '''
        dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
        du/dt = (0.02*(0.2*v - u))/ms : 1
        I : 1
        '''
        self._snn_reservoir = NeuronGroup(number_of_neurons, eqs, threshold='v>30', reset ='''
        v = -65
        u = u + 6
        ''')
        self._snn_reservoir.I = 0
        self._snn_reservoir.v = -70
        self._snn_reservoir.u = -14
        synapses = Synapses(self._snn_reservoir, self._snn_reservoir, pre='v_post += 20')
        synapses.connect('(i!=j and rand()<0.6) or (i==j and rand()<0.2)')

        # TODO rethink if you need recording here if it will not affect
        # performance
        self._membrane_potential_monitor = StateMonitor(self._snn_reservoir,
                                                        'v', record=True)
        self._membrane_recovery_monitor = StateMonitor(self._snn_reservoir,
                                                        'u', record=True)

    def _set_current(self, current):
        # self._snn_reservoir[:1].I = 14
        self._snn_reservoir.I = current

    def run_simulation(self,millisec):
        run(millisec*ms)





# plot(M.t/ms, M.v[0], '-b', lw=2, label='N0: membrane potential')
# plot(M.t/ms, Mu.u[0], '-g', label='N0: membrane recovery')
# plot(M.t/ms, M.v[1], '-r', lw=2, label='N1: membrane potential')
# plot(M.t/ms, Mu.u[1], '-y', label='N1: membrane recovery')
# xlabel('Time (ms)')
# ylabel('v')
# legend(loc='best')
# show()

# for i, j in zip(S.i, S.j):
#     if(i == j):
#         print 'Source: ' + str(i) + ' Target: ' + str(j)
#
# print np.amax(M.v[0]) # max value around 415
# print np.amin(M.v[0]) # min value around -245