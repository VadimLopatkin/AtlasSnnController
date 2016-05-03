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
import numpy as np

from datetime import datetime
from lib.ReservoirNetworkController import ReservoirNetworkController
from lib.AtlasJointsInfo import AtlasJointsInfo


class SpikingNeuralNetwork:
    INPUT_LAYER_SIZE = 28
    OUTPUT_LAYER_SIZE = 28
    SIMULATION_TIME_INTERVAL = 15 # msec
    RESERVOIR_NETWORK_SIZE = 500
    MAX_INPUT_RATE = 1000
    MIN_INPUT_RATE = 50

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
        snn_controller = ReservoirNetworkController(
                self.RESERVOIR_NETWORK_SIZE, self.INPUT_LAYER_SIZE)
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

    def _normalize_position_input(self, position_input):
        result = []
        for i in xrange(len(position_input)):
            result.append((position_input[i] -
                           self._joints_info_provider.get_min_value_for_joint(
                               i)) / (
                              self._joints_info_provider.get_max_value_for_joint(
                                  i) -
                              self._joints_info_provider.get_min_value_for_joint(
                                  i)))
        # output values are between 0 and 1
        return result

    def get_input_layer_values(self):
        return self._input_layer

    def _apply_poisson_group_input(self):
        firing_rates = np.zeros(len(self._input_layer))
        for i in xrange(len(self._input_layer)):
            firing_rates[i] = self._convert_to_rate(self._input_layer[i])
        self._hidden_layer.set_poisson_group_rates(firing_rates)

    def _convert_to_rate(self, input):
        # input values are between 0 and 1 (after normalization)
        rate = input*(self.MAX_INPUT_RATE - self.MIN_INPUT_RATE) + \
               self.MIN_INPUT_RATE
        return rate

    def _decode_snn_output(self):
        firing_rates = self._hidden_layer.get_reservoir_firing_rates_output()
        # print "SpikingNeuralNetwork._decode_snn_output(): " \
        #       "np.amax(firing_rates) = " + str(
        #         np.amax(firing_rates))
        # print "SpikingNeuralNetwork._decode_snn_output(): " \
        #       "np.amin(firing_rates) = " + str(
        #         np.amin(firing_rates))
        self._compute_activations_from_reservoir(firing_rates)
        for i in xrange(self.OUTPUT_LAYER_SIZE):
            self._output_layer[i] = self._output_layer_activations[i]

    def _normalize_firing_rates_output(self, input_value):
        max_value = self.MAX_INPUT_RATE
        min_value = self.MIN_INPUT_RATE
        result = []
        for i in xrange(len(input_value)):
            result.append((input_value[i] - min_value) / (max_value - min_value))
        return result

    def get_output_layer_values(self):
        return self._output_layer

    def _compute_activations_from_reservoir(self, firing_rates):
        for i in xrange(len(self._hidden_layer_neuron_mapping)):
            one_neuron_mapping = self._hidden_layer_neuron_mapping[i]
            z = 0
            for c in xrange(len(one_neuron_mapping)):
                hidden_neuron_index = one_neuron_mapping[c]
                rate = firing_rates[hidden_neuron_index]
                # TODO think of a more elegant way to deal with it
                if rate == 0:
                    rate = 0.0001
                # TODO: computing activation from reservoir as
                # rate/self.MAX_INPUT_RATE is
                # quite questionable!!!
                z = z + self._hidden_layer_weights[hidden_neuron_index][i]*(
                    rate/self.MAX_INPUT_RATE)
            z = z + self._hidden_layer_biases[i]
            activation = self._sigmoid(z)
            self._output_layer_activations[i] = activation

    def _initialize_hidden_layer_neuron_mapping(self):
        mapping = []
        for i in xrange(self.OUTPUT_LAYER_SIZE):
            # TODO should be np.random.randint(10)
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
        # print "entering SpikingNeuralNetwork.set_hidden_layer_biases"
        self._hidden_layer_biases = hidden_layer_biases
        # print "leaving SpikingNeuralNetwork.set_hidden_layer_biases"

    def recalculate_output_layer(self):
        self._decode_snn_output()
