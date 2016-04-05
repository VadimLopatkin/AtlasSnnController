import rospy
import numpy as np

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
        self._joint_commands_publisher = rospy.Publisher(
                '/atlas/joint_commands', JointCommands, queue_size=1)
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
            self._convert_output()
            self._send_walking_command_to_atlas()
        delta = datetime.now() - start_time
        print "processing time: " + str(delta.seconds) + "." + str(
                delta.microseconds/1000) + "\n"

    def get_current_state(self):
        return self._current_state

    def get_network(self):
        return self._network

    def _convert_output(self):
        # print "entering AtlasController._convert_output()"
        result = []
        network_output = self._network.get_output_layer_values()
        for i in xrange(len(network_output)):
            network_output_item = network_output[i]
            if network_output_item > 1:
                # TODO: this case is, probably impossible as
                # network_output_item is a sigmoid function output when using
                #  artificial neurons in output layer
                print "network_output_item > 1 : " + str(network_output_item)
                result.append(
                        self._joints_info_provider.get_max_value_for_joint(i))
            elif network_output_item < 0:
                # TODO: this case is, probably impossible as
                # network_output_item is a sigmoid function output when using
                #  artificial neurons in output layer
                print "network_output_item < 0 : " + str(network_output_item)
                result.append(
                        self._joints_info_provider.get_min_value_for_joint(i))
            else:
                normalized_value = (
                    self._joints_info_provider.get_max_value_for_joint(i) -
                    self._joints_info_provider.get_min_value_for_joint(
                            i))*network_output_item+self._joints_info_provider.get_min_value_for_joint(i)
                # print "normalized_value = " + str(normalized_value)
                result.append(normalized_value)
        self._output = result
        # print "leaving AtlasController._convert_output()"

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

    def get_output_layer_activations(self):
        # TODO: temporary validation
        output_layer_activations = self._network.get_output_layer_values()
        max_value = np.amax(output_layer_activations)
        if max_value>1:
            print "AtlasController.get_output_layer_activations: max_value = " \
                  "" + str(max_value)
        min_value = np.amin(output_layer_activations)
        if min_value<0:
            print "AtlasController.get_output_layer_activations: min_value = " + str(min_value)
        return output_layer_activations

    def recalculate_output_layer(self):
        self._network.recalculate_output_layer()
        self._convert_output()

    def set_learning_mode(self, mode):
        self._learning_mode = mode

    def _send_walking_command_to_atlas(self):
        atlasJointNames = []
        n = len(self._current_state.position)
        for i in xrange(n):
            atlasJointNames.append(
                    self._joints_info_provider.get_full_name_for_joint(i))
        command = JointCommands()
        command.position     = np.zeros(n)
        command.velocity     = np.zeros(n)
        command.effort       = np.zeros(n)
        command.kp_position  = np.zeros(n)
        command.ki_position  = np.zeros(n)
        command.kd_position  = np.zeros(n)
        command.kp_velocity  = np.zeros(n)
        command.i_effort_min = np.zeros(n)
        command.i_effort_max = np.zeros(n)
        for i in xrange(n):
            command.kp_position[i]  = rospy.get_param(
                    'atlas_controller/gains/' +
                    self._joints_info_provider.get_short_name_for_joint(i) +
                    '/p')
            command.ki_position[i]  = rospy.get_param(
                    'atlas_controller/gains/' +
                    self._joints_info_provider.get_short_name_for_joint(i) +
                    '/i')
            command.kd_position[i]  = rospy.get_param(
                    'atlas_controller/gains/' +
                    self._joints_info_provider.get_short_name_for_joint(i) +
                    '/d')
            command.i_effort_max[i] = rospy.get_param(
                    'atlas_controller/gains/' +
                    self._joints_info_provider.get_short_name_for_joint(i) +
                    '/i_clamp')
            command.i_effort_min[i] = -command.i_effort_max[i]
        command.position = self._output
        self._joint_commands_publisher.publish(command)
