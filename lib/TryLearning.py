from __future__ import division
from brian2 import *
import numpy as np

network = Network()
poisson_group = PoissonGroup(1, 5 * Hz)
network.add(poisson_group)

izhikevich_equation = '''
        dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
        du/dt = (0.02*(0.2*v - u))/ms : 1
        I : 1
        '''
snn_reservoir = NeuronGroup(10, izhikevich_equation,
                                          threshold='v>30', reset='''
        v = -65
        u = u + 6
        ''')
snn_reservoir.I = 50
snn_reservoir.v = -70
snn_reservoir.u = -14
network.add(snn_reservoir)

output_layer_neurons = NeuronGroup(1, izhikevich_equation,
                                          threshold='v>30', reset='''
        v = -65
        u = u + 6
        ''')
output_layer_neurons.I = 0
output_layer_neurons.v = -70
output_layer_neurons.u = -14
network.add(output_layer_neurons)

input_synapses = Synapses(poisson_group, snn_reservoir, pre='v+=30')
# pre code can later be changed via self._input_synapses.pre.code =
# 'v+=30'
input_synapses.connect(True)
network.add(input_synapses)

synapses = Synapses(snn_reservoir, snn_reservoir, pre='''
I=5
v+=150
''')
synapses.delay='j*rand()*ms'
synapses.connect('i!=j and rand()<0.1')
synapses.connect('i==j and rand()<0.5')
network.add(synapses)

output_synapses = Synapses(snn_reservoir, output_layer_neurons,'''
w : 1
current : 1
''',
                                         pre='''
                                         v_post += w
                                         I_post *= current
                                         ''')
# output_synapses = Synapses(snn_reservoir, output_layer_neurons, pre='v_post '
#                                                                     '+= 20')
output_synapses.connect(True)
network.add(output_synapses)
output_synapses.w = 20
output_synapses.current = 1


hidden_layer_spike_monitor = SpikeMonitor(snn_reservoir)
network.add(hidden_layer_spike_monitor)
output_layer_spike_monitor = SpikeMonitor(output_layer_neurons)
network.add(output_layer_spike_monitor)

poisson_group.rates.set_item(0,150*Hz)
network.run(200 * ms)
poisson_group.rates.set_item(0,0*Hz)
network.run(500 * ms)

spike_trains = hidden_layer_spike_monitor.spike_trains()
firing_rates = []
for i in xrange(len(spike_trains)):
    firing_rates.append((1000 * len(spike_trains[i])) / 700)

max_value = np.amax(firing_rates)
min_value = np.amin(firing_rates)
print min_value
print max_value
print len(synapses)

output_spiking_trains = output_layer_spike_monitor.spike_trains()
output_firing_rate = 1000 * len(output_spiking_trains[0]) / 700
print output_firing_rate

################################################################################
print '#####################################'

desired_rate = 50

for i in xrange(len(output_synapses)):
    hidden_neuron_idx = output_synapses.i[i]
    hidden_neuron_rate = firing_rates[hidden_neuron_idx]
    weighting_factor = abs(1 -(hidden_neuron_rate - desired_rate) /
                           desired_rate)**2
    print "weighting_factor: " + str(weighting_factor)
    print output_synapses.current[i]
    output_synapses.current[i] *= weighting_factor
    print output_synapses.current[i]
    # output_layer_neurons[i].I *= weighting_factor

network.remove(hidden_layer_spike_monitor)
hidden_layer_spike_monitor = SpikeMonitor(snn_reservoir)
network.add(hidden_layer_spike_monitor)
network.remove(output_layer_spike_monitor)
output_layer_spike_monitor = SpikeMonitor(output_layer_neurons)
network.add(output_layer_spike_monitor)



network.run(500 * ms)

spike_trains = hidden_layer_spike_monitor.spike_trains()
firing_rates = []
for i in xrange(len(spike_trains)):
    firing_rates.append((1000 * len(spike_trains[i])) / 500)

max_value = np.amax(firing_rates)
min_value = np.amin(firing_rates)

output_spiking_trains = output_layer_spike_monitor.spike_trains()
output_firing_rate = 1000 * len(output_spiking_trains[0]) / 500

print min_value
print max_value
print output_firing_rate