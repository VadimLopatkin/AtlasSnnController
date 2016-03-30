from __future__ import division
import numpy as np

from datetime import datetime
from lib.ReservoirNetworkController import ReservoirNetworkController
from lib.AtlasJointsInfo import AtlasJointsInfo


class SpikingNeuralNetwork:
    INPUT_LAYER_SIZE = 277 # probably smaller, as k_effort is just flags (it's
    #  28 of them)
    OUTPUT_LAYER_SIZE = 28
    SIMULATION_TIME_INTERVAL = 150 # in msec
    RESERVOIR_NETWORK_SIZE = 500

    def __init__(self):
        self._output_layer_activations = np.zeros(self.OUTPUT_LAYER_SIZE)
        self._hidden_layer_weights = np.zeros([self.RESERVOIR_NETWORK_SIZE,
                                               self.OUTPUT_LAYER_SIZE])
        self._hidden_layer_neuron_mapping =  \
            self._initialize_hidden_layer_neuron_mapping()
        for i in xrange(len(self._hidden_layer_neuron_mapping)):
            one_neuron_mapping = self._hidden_layer_neuron_mapping[i]
            for c in xrange(len(one_neuron_mapping)):
                hidden_neuron_index = one_neuron_mapping[c]
                self._hidden_layer_weights[hidden_neuron_index][i] = \
                    np.random.rand(1)[0]
        self._hidden_layer_biases = np.random.rand(self.OUTPUT_LAYER_SIZE)
        self._joints_info_provider = AtlasJointsInfo()
        self._input_layer = np.zeros(self.INPUT_LAYER_SIZE)
        self._hidden_layer = self._initialize_reservoir_network()
        self._output_layer = np.zeros(self.OUTPUT_LAYER_SIZE)

    def _initialize_reservoir_network(self):
        snn_controller = ReservoirNetworkController(self.RESERVOIR_NETWORK_SIZE)
        return snn_controller

    def process_input(self, state):
        # TODO so far we are using only position, eventually should use more
        # parameters
        start_time = datetime.now()
        self._set_position_values(self._normalize_position_input(state.position))
        stop_time = datetime.now()
        delta = stop_time - start_time
        print "converting input in: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000)
        start_time = stop_time
        self._apply_poisson_group_input()
        stop_time = datetime.now()
        delta = stop_time - start_time
        print "applying Poisson group in: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000)
        start_time = stop_time
        self._hidden_layer.run_simulation(self.SIMULATION_TIME_INTERVAL)
        stop_time = datetime.now()
        delta = stop_time - start_time
        print "simulation running in: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000)
        start_time = stop_time
        self._decode_snn_output()
        stop_time = datetime.now()
        delta = stop_time - start_time
        print "decoding output in: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000)

    def _set_position_values(self, position):
        for i in xrange(len(position)):
            self._input_layer[i] = position[i]

    def _normalize_position_input(self, input_value):
        result = []
        for i in xrange(len(input_value)):
            result.append((input_value[i] -
                           self._joints_info_provider.get_min_value_for_joint(
                               i)) / (
                              self._joints_info_provider.get_max_value_for_joint(
                                  i) -
                              self._joints_info_provider.get_min_value_for_joint(
                                  i)))
        return result

    def get_input_layer_values(self):
        return self._input_layer

    def _apply_poisson_group_input(self):
        firing_rates = np.zeros(len(self._input_layer))
        for i in xrange(len(self._input_layer)):
            firing_rates[i] = self._convert_to_rate(self._input_layer[i])
        self._hidden_layer.set_poisson_group_rates(firing_rates)

    def _convert_to_rate(self, input):
        # biologically plausible is between 5 and 100 Hz
        rate = input*95 + 5
        return rate

    def _decode_snn_output(self):
        firing_rates = self._hidden_layer.get_reservoir_firing_rates_output()
        firing_rates_normalized = self._normalize_firing_rates_output(
            firing_rates)
        # len(firing_rates_normalized) is supposed to be equal to
        # RESERVOIR_NETWORK_SIZE
        self._compute_activations_from_reservoir(firing_rates_normalized)
        for i in xrange(self.OUTPUT_LAYER_SIZE):
            self._output_layer[i] = self._output_layer_activations[i]

    def _normalize_firing_rates_output(self, input_value):
        # biologically plausible is between 5 and 100 Hz
        max_value = 100
        min_value = 5
        result = []
        for i in xrange(len(input_value)):
            result.append((input_value[i] - min_value) / (max_value - min_value))
        return result

    def get_output_layer_values(self):
        return self._output_layer

    def _compute_activations_from_reservoir(self, firing_rates_normalized):
        for i in xrange(len(self._hidden_layer_neuron_mapping)):
            one_neuron_mapping = self._hidden_layer_neuron_mapping[i]
            z = 0
            for c in xrange(len(one_neuron_mapping)):
                hidden_neuron_index = one_neuron_mapping[c]
                rate = firing_rates_normalized[hidden_neuron_index]
                # TODO: computing activation from reservoir as 1/rate is
                # quite questionable!!!
                z = z + self._hidden_layer_weights[hidden_neuron_index][i]*(
                    1/rate)
            z = z + self._hidden_layer_biases[i]
            activation = self._sigmoid(z)
            self._output_layer_activations[i] = activation

    def _initialize_hidden_layer_neuron_mapping(self):
        mapping = []
        for i in xrange(self.OUTPUT_LAYER_SIZE):
            connected_neurons_num = np.random.randint(1,10,1)[0]
            one_neuron_mapping = np.random.randint(1,
                                                self.RESERVOIR_NETWORK_SIZE,
                                                   connected_neurons_num)
            mapping.append(one_neuron_mapping)
        return mapping

    def _sigmoid(self, z):
        return 1.0/(1.0+np.exp(-z))

    def get_hidden_layer_weights_for_output_neuron(self, neuron_idx):
        # print "entering get_hidden_layer_weights_for_output_neuron"
        hidden_layer_weights_T = self._hidden_layer_weights.T
        # print "len(hidden_layer_weights_T[neuron_idx]) = " + str(len(
        #         hidden_layer_weights_T[neuron_idx]))
        # print "hidden_layer_weights_T[neuron_idx] : " + str(
        #         hidden_layer_weights_T[neuron_idx])
        # print "leaving get_hidden_layer_weights_for_output_neuron"
        return hidden_layer_weights_T[neuron_idx]

    def get_hidden_layer_firing_rates(self):
        return self._hidden_layer.get_reservoir_firing_rates_output()

    def get_mapping_for_output_neuron(self, neuron_idx):
        return self._hidden_layer_neuron_mapping[neuron_idx]

    def get_hidden_layer_biases(self):
        return self._hidden_layer_biases

    def set_hidden_layer_weights_for_neuron(self, neuron_idx,
                                            hidden_layer_weights):
        # print "entering set_hidden_layer_weights_for_neuron"
        # print "len(self._hidden_layer_weights.T[neuron_idx]) = " + str(len(
        #         self._hidden_layer_weights.T[neuron_idx]))
        # print "len(hidden_layer_weights) = " + str(len(hidden_layer_weights))
        self._hidden_layer_weights.T[neuron_idx] = hidden_layer_weights
        # print "leaving set_hidden_layer_weights_for_neuron"

    def set_hidden_layer_biases(self, hidden_layer_biases):
        print "entering set_hidden_layer_biases"
        self._hidden_layer_biases = hidden_layer_biases
        print "leaving set_hidden_layer_biases"

