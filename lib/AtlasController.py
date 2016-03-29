import rospy

from atlas_msgs.msg import AtlasState, AtlasSimInterfaceState
from osrf_msgs.msg import JointCommands
from datetime import datetime

from lib.SpikingNeuralNetwork import SpikingNeuralNetwork
from lib.AtlasControllerTrainer import AtlasControllerTrainer
from lib.AtlasJointsInfo import AtlasJointsInfo


class AtlasController:
    def __init__(self, learning_mode=False):
        self._initializing = True
        self._joints_info_provider = AtlasJointsInfo()
        self._learning_mode = learning_mode
        if self._learning_mode:
            self._current_state = AtlasState()
        rospy.init_node('snn_atlas_controller')
        self._subscriber = rospy.Subscriber('atlas/atlas_state', AtlasState,
                                            self._state_cb, queue_size=1)
        self._network = SpikingNeuralNetwork()
        self._initializing = False

    def _state_cb(self, msg):
        start_time = datetime.now()
        if self._initializing:
            self._current_state = msg
            return
        if self._learning_mode:
            self._previous_state = self._current_state
            self._current_state = msg
            self._network.process_input(self._previous_state)
            self._convert_output()
            AtlasControllerTrainer.train_the_network(self)
        else:
            self._current_state = msg
            self._network.process_input(self._current_state)
        delta = datetime.now() - start_time
        print "processing time: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000) + "\n"

    def get_current_state(self):
        return self._current_state

    def get_network(self):
        return self._network

    def _convert_output(self):
        result = []
        network_output = self._network.get_output_layer_values()
        for i in xrange(len(network_output)):
            network_output_item = network_output[i]
            if network_output_item > 1:
                result.append(
                        self._joints_info_provider.get_max_value_for_joint(i))
            elif network_output_item < 0:
                result.append(
                        self._joints_info_provider.get_min_value_for_joint(i))
            else:
                normalized_value = (
                    self._joints_info_provider.get_max_value_for_joint(i) -
                    self._joints_info_provider.get_min_value_for_joint(
                            i))*network_output_item+self._joints_info_provider.get_min_value_for_joint(i)
                result.append(normalized_value)
        self._output = result

    def get_output(self):
        return self._output

    def get_hidden_layer_weights_for_output_neuron(self,neuron_idx):
        return self._network.get_hidden_layer_weights_for_output_neuron(
                neuron_idx)

    def get_hidden_layer_firing_rates(self):
        return self._network.get_hidden_layer_firing_rates()

    def get_hidden_layer_biases(self):
        return self._network.get_hidden_layer_biases()

    def get_mapping_for_output_neuron(self, neuron_idx):
        return self._network.get_mapping_for_output_neuron(neuron_idx)

    def set_hidden_layer_weights_for_neuron(self,neuron_idx,
                                            hidden_layer_weights):
        self._network.set_hidden_layer_weights_for_neuron(neuron_idx,
                                                          hidden_layer_weights)

    def set_hidden_layer_biases(self,hidden_layer_biases):
        self._network.set_hidden_layer_biases(hidden_layer_biases)


def atlas_command_reader(msg):
    print "enter"
    print msg


def main():
    rospy.init_node('atlas_controller')
    subscriber = rospy.Subscriber('atlas/joint_commands', JointCommands,
                                  atlas_command_reader)
    print "start"
    rospy.spin()
    print "finish"


if __name__ == '__main__':
    main()
