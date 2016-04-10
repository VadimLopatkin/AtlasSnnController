# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

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

hidden_layer_spike_monitor = SpikeMonitor(snn_reservoir)
network.add(hidden_layer_spike_monitor)

poisson_group.rates.set_item(0,150*Hz)
network.run(200 * ms)
poisson_group.rates.set_item(0,0*Hz)
network.run(500 * ms)

spike_trains = hidden_layer_spike_monitor.spike_trains()
firing_rates = []
for i in xrange(len(spike_trains)):
    firing_rates.append((1000 * len(spike_trains[i])) / 700)

hidden_layer_weights = np.random.rand(len(firing_rates))
# hidden_layer_bias = np.random.randint(1,100,1)[0]
hidden_layer_bias = np.random.rand(1)[0]
# for i in xrange(len(firing_rates)):
#     z = hidden_layer_weights[i] * firing_rates[i] + hidden_layer_biases[i]
#     print str(i) + ": z = " + str(z)
#     a = 1/(1+np.exp(-z))
#     print str(i) + ": a = " + str(a) + "\n"
#     hidden_layer_activations.append(a)
z = 0
for i in xrange(len(firing_rates)):
    z = z + hidden_layer_weights[i] * (1/firing_rates[i])
z += hidden_layer_bias
a = 1.0/(1.0+np.exp(-z))
print "a = " + str(a)
desired_activation_value = 0.17
eta = 3.0
for i in xrange(20):
    delta = (a - desired_activation_value)*a*(1-a)
    print str(i) + ": delta = " + str(delta)
    nabla_b = delta
    for c in xrange(len(hidden_layer_weights)):
        nabla_w = delta*(1/firing_rates[c])
        hidden_layer_weights[c] = hidden_layer_weights[c] - eta*nabla_w
    hidden_layer_bias = hidden_layer_bias - eta*nabla_b
    z = 0
    for c in xrange(len(firing_rates)):
        z = z + hidden_layer_weights[c] * (1/firing_rates[c])
    z += hidden_layer_bias
    a = 1.0/(1.0+np.exp(-z))
    print str(i) + ": a = " + str(a)
    print "\n"