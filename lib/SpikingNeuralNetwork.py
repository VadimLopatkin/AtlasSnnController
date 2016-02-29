import numpy as np

from lib.IzhikevichController import IzhikevichController


class SpikingNeuralNetwork():
    INPUT_LAYER_SIZE = 277 # probably smaller as k_effort is just flags (it's
    #  28 of them)
    OUTPUT_LAYER_SIZE = 27

    def __init__(self):
        self._input_layer = np.zeros(self.INPUT_LAYER_SIZE)
        self._hidden_layer = self._initialize_reservoir_network()
        self._output_layer = np.zeros(self.OUTPUT_LAYER_SIZE)

    def _initialize_reservoir_network(self):
        snn_controller = IzhikevichController(500)
        return snn_controller

    def set_input_layer_state(self, state):
        self._set_position_values(state.position)
        self._normalize_input_layer()

    def _set_position_values(self, position):
        for i in xrange(len(position)):
            self._input_layer[i] = position[i]

    def _normalize_input_layer(self):
        max_value = np.amax(self._input_layer)
        min_value = np.amin(self._input_layer)
        for i in xrange(len(self._input_layer)):
            self._input_layer[i] = (self._input_layer[i] - min_value)/(
                max_value - min_value)

    def get_input_layer_values(self):
        return self._input_layer