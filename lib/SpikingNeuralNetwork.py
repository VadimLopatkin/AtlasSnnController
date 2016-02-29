import numpy as np


class SpikingNeuralNetwork():
    INPUT_LAYER_SIZE = 277 # probably smaller as k_effort is just flags (it's
    #  28 of them)
    INPUT_LAYER_SIZE = 277

    def __init__(self):
        self._input_layer = np.zeros(self.INPUT_LAYER_SIZE)
        self._hidden_layer = self._initialize_reservoir_network()
        self._output_layer = np.zeros(self.OUTPUT_LAYER_SIZE)

    def _initialize_reservoir_network(self):
        snn_controller = IzhikevichController()
        return None