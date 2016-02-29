from brian2 import *
import numpy as np


eqs = '''
dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
du/dt = (0.02*(0.2*v - u))/ms : 1
I : 1
'''

SnnReservoir = NeuronGroup(50, eqs, threshold='v>30', reset ='''
v = -65
u = u + 6
''')
SnnReservoir.I = 0
SnnReservoir.v = -70
SnnReservoir.u = -14

S = Synapses(SnnReservoir, SnnReservoir, pre='v_post += 20')
S.connect('(i!=j and rand()<0.6) or (i==j and rand()<0.2)')






M = StateMonitor(SnnReservoir, 'v', record=True)
Mu = StateMonitor(SnnReservoir, 'u', record=True)
run(10*ms)
SnnReservoir[:1].I = 14
run(9000*ms)
# plot(M.t/ms, M.v[0], '-b', lw=2, label='N0: membrane potential')
# plot(M.t/ms, Mu.u[0], '-g', label='N0: membrane recovery')
# plot(M.t/ms, M.v[1], '-r', lw=2, label='N1: membrane potential')
# plot(M.t/ms, Mu.u[1], '-y', label='N1: membrane recovery')
# xlabel('Time (ms)')
# ylabel('v')
# legend(loc='best')
# show()

for i, j in zip(S.i, S.j):
    if(i == j):
        print 'Source: ' + str(i) + ' Target: ' + str(j)

print np.amax(M.v[0]) # max value around 415
print np.amin(M.v[0]) # min value around -245