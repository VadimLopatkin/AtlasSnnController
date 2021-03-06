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
from datetime import datetime

from lib.FiringRatesSaver import FiringRatesSaver


class ReservoirNetworkController:

    def __init__(self, number_of_neurons, input_layer_size):
        self._firing_rates_recorder = FiringRatesSaver()
        self._input_layer_size = input_layer_size
        self._hidden_layer_spike_monitor = None
        self._output_layer_spike_monitor = None
        self._network = Network()
        self._initialize_neuron_groups(number_of_neurons)

        self._initialize_input_synapses()
        self._initialize_reservoir_synapses()

        self._initialize_monitoring()

    def _initialize_reservoir_synapses(self):
        self._synapses = Synapses(self._snn_reservoir, self._snn_reservoir,
                                  pre='v_post += 20')
        self._synapses.connect('(i!=j and rand()<0.008) or (i==j and rand('
                               ')<0.2)')
        self._network.add(self._synapses)

    def _initialize_input_synapses(self):
        self._input_synapses = Synapses(self._poisson_group,
                                        self._snn_reservoir,
                                        pre='v_post+=30')
        # pre code can later be changed via self._input_synapses.pre.code =
        # 'v+=30'
        self._input_synapses.connect(
            '(i!=j and rand()<0.005) or (i==j and rand('
            ')<0.9)')
        self._network.add(self._input_synapses)

    def _initialize_neuron_groups(self, number_of_neurons):
        self._initialize_poisson_input_layer()
        self._izhikevich_equation = '''
        dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
        du/dt = (0.02*(0.2*v - u))/ms : 1
        I : 1
        '''
        self._initialize_hidden_neuron_layer(number_of_neurons)

    def _initialize_hidden_neuron_layer(self, number_of_neurons):
        self._snn_reservoir = NeuronGroup(number_of_neurons,
                                          self._izhikevich_equation,
                                          threshold='v>30', reset='''
        v = -65
        u = u + 6
        ''')
        # TODO we might need to add current (14) during the simulation
        self._snn_reservoir.I = 0
        self._snn_reservoir.v = -70
        self._snn_reservoir.u = -14
        self._network.add(self._snn_reservoir)

    def _initialize_poisson_input_layer(self):
        self._poisson_group = PoissonGroup(self._input_layer_size, 5 * Hz)
        self._network.add(self._poisson_group)

    def _set_current(self, current):
        # self._snn_reservoir[:1].I = 14
        self._snn_reservoir.I = current

    def run_simulation(self, millisec):
        self._simulation_run_interval = millisec
        # TODO: check if it's a reason for slow performance
        self._initialize_monitoring()
        self._network.run(millisec * ms)

    def set_poisson_group_rates(self, firing_rates):
        for i in xrange(len(firing_rates)):
            self._poisson_group.rates.set_item(i,firing_rates[i]*Hz)

    def _initialize_monitoring(self):
        start_time = datetime.now()
        if(self._hidden_layer_spike_monitor != None):
            self._network.remove(self._hidden_layer_spike_monitor)
            self._hidden_layer_spike_monitor = None
        self._hidden_layer_spike_monitor = SpikeMonitor(
            self._snn_reservoir)
        self._network.add(self._hidden_layer_spike_monitor)
        delta = datetime.now() - start_time
        print "initializing monitoring in: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000)

    def get_hidden_reservoir_firing_rates(self):
        firing_rates = self.get_firing_rates_from_monitor(
                self._hidden_layer_spike_monitor)
        return firing_rates

    def get_firing_rates_from_monitor(self, spike_monitor):
        spike_trains = spike_monitor.spike_trains()
        firing_rates = []
        for i in xrange(len(spike_trains)):
            firing_rates.append((1000 * len(spike_trains[i])) /
                                self._simulation_run_interval)
        # TODO these max and min variables are used for convenient debugging
        # max = np.amax(firing_rates)
        # min = np.amin(firing_rates)
        self._firing_rates_recorder.save_firing_rates(firing_rates)
        return firing_rates

    def get_reservoir_firing_rates_output(self):
        # TODO maybe we could compute the firing rates only ones each time
        # the simulation step is over and save it in an array to access it
        # multiple times afterwards
        firing_rates = self.get_firing_rates_from_monitor(
                self._hidden_layer_spike_monitor)
        return firing_rates