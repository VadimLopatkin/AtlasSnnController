import numpy as np

from lib.ReservoirNetworkController import ReservoirNetworkController


class SpikingNeuralNetwork():
    INPUT_LAYER_SIZE = 277 # probably smaller, as k_effort is just flags (it's
    #  28 of them)
    OUTPUT_LAYER_SIZE = 27
    SIMULATION_TIME_INTERVAL = 150 # in msec
    RESERVOIR_NETWORK_SIZE = 500

    def __init__(self):
        self._input_layer = np.zeros(self.INPUT_LAYER_SIZE)
        self._hidden_layer = self._initialize_reservoir_network()
        self._output_layer = np.zeros(self.OUTPUT_LAYER_SIZE)

    def _initialize_reservoir_network(self):
        snn_controller = ReservoirNetworkController(self.RESERVOIR_NETWORK_SIZE)
        return snn_controller

    def process_input(self, state):
        # TODO so far we are using only position, eventually should use more
        # parameters
        self._set_position_values(self._normalize_input(state.position))
        self._apply_poisson_group_input()
        self._hidden_layer.run_simulation(self.SIMULATION_TIME_INTERVAL)
        self._decode_snn_output()

    def _set_position_values(self, position):
        for i in xrange(len(position)):
            self._input_layer[i] = position[i]

    def _normalize_input(self, input):
        max_value = np.amax(input)
        min_value = np.amin(input)
        result = []
        for i in xrange(len(input)):
            result.append((input[i] - min_value)/(max_value - min_value))
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
        firing_rates = self._hidden_layer.get_reservoir_firing_rates()
