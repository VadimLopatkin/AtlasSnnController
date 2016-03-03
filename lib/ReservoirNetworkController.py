from brian2 import *
import numpy as np


class ReservoirNetworkController():
    _INPUT_LAYER_SIZE = 277
    _SPIKE_WINDOW = 150

    def __init__(self, number_of_neurons):
        self._network = Network()
        self._poisson_group = PoissonGroup(self._INPUT_LAYER_SIZE, 5 * Hz)
        self._network.add(self._poisson_group)
        eqs = '''
        dv/dt = (0.04*(v**2)+5*v+140 - u + I)/ms : 1
        du/dt = (0.02*(0.2*v - u))/ms : 1
        I : 1
        '''
        self._snn_reservoir = NeuronGroup(number_of_neurons, eqs,
                                          threshold='v>30', reset='''
        v = -65
        u = u + 6
        ''')
        self._snn_reservoir.I = 0
        self._snn_reservoir.v = -70
        self._snn_reservoir.u = -14
        self._network.add(self._snn_reservoir)
        self._synapses = Synapses(self._snn_reservoir, self._snn_reservoir,
                            pre='v_post += 20')
        self._synapses.connect('(i!=j and rand()<0.6) or (i==j and rand()<0.2)')
        self._network.add(self._synapses)
        self._input_synapses = Synapses(self._poisson_group, self._snn_reservoir,
                                  pre='v+=0.1')
        self._input_synapses.connect('(i!=j and rand()<0.005) or (i==j and rand('
                               ')<0.9)')
        self._network.add(self._input_synapses)
        self._initialize_monitoring()

    def _set_current(self, current):
        # self._snn_reservoir[:1].I = 14
        self._snn_reservoir.I = current

    def run_simulation(self, millisec):
        self._network.run(millisec * ms)

    def set_poisson_group_rates(self, firing_rates):
        for i in xrange(len(firing_rates)):
            self._poisson_group.rates.set_item(i,firing_rates[i]*Hz)

    def _initialize_monitoring(self):
        self._hidden_layer_spike_monitor = SpikeMonitor(
            self._snn_reservoir)
        self._network.add(self._hidden_layer_spike_monitor)

    def get_reservoir_firing_rates(self):
        spike_trains = self._hidden_layer_spike_monitor
        firing_rates = []
        for i in xrange(len(spike_trains)):
            firing_rates.append((1000 * self._count_latest_spikes(
                    spike_trains[i])) / self._SPIKE_WINDOW)
        return firing_rates

    def _count_latest_spikes(self, neuron_spikes):
        n_latest_spikes = 0
        if(len(neuron_spikes)>0):
            n = -1
            time_after_spike = (self._network.t-neuron_spikes[n])/ms
            while((time_after_spike < self._SPIKE_WINDOW) and (abs(n)<=len(
                    neuron_spikes))):
                n_latest_spikes+=1
                n-=1
        return n_latest_spikes
