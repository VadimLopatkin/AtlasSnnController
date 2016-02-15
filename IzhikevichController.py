from brian2 import *


eqs = '''
dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
du/dt = (0.02*(0.2*v - u))/ms : 1
I : 1
'''

# v0 = -70
# u0 = -14

G = NeuronGroup(2, eqs, threshold='v>30', reset = '''
v = -65
u = u + 6
''')
G.I = [0, 0]
G.v = [-70, -70]
G.u = [-14,-14]

S = Synapses(G, G, pre='v_post += 20')
S.connect(0, 1)
M = StateMonitor(G, 'v', record=True)
Mu = StateMonitor(G, 'u', record=True)
run(10*ms)
G.I = [14, 0]
run(90*ms)
plot(M.t/ms, M.v[0], '-b', lw=2, label='N0: membrane potential')
plot(M.t/ms, Mu.u[0], '-g', label='N0: membrane recovery')
plot(M.t/ms, M.v[1], '-r', lw=2, label='N1: membrane potential')
plot(M.t/ms, Mu.u[1], '-y', label='N1: membrane recovery')
xlabel('Time (ms)')
ylabel('v')
legend(loc='best')
show()