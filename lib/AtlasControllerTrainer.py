from __future__ import division
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState
import rospy
import numpy as np

from lib.AtlasJointsInfo import AtlasJointsInfo


class AtlasControllerTrainer:
    def __init__(self):
        self._atlas_joints_info_provider = AtlasJointsInfo()
        self._current_behavior = AtlasSimInterfaceCommand.NONE
        self._sim_interface_subscriber = rospy.Subscriber(
                'atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            self._sim_int_cb)
        self._trainings_counter = 0
        self._calls_counter = 0

    @classmethod
    def train_the_network(cls, controller):
        global INSTANCE
        INSTANCE.increase_calls_counter()
        if INSTANCE.get_current_behavior() == AtlasSimInterfaceCommand.WALK:
            print str(INSTANCE.get_trainings_counter()) + ": network is being " \
                                                          "" \
                                                          "trained"
            controller_output = controller.get_output()
            print "validating controller output"
            cls._validate_output(controller_output)
            expected_output = controller.get_current_state().position
            print "validating expected output"
            cls._validate_output(expected_output)
            absolute_deltas = cls._compute_absolute_deltas(controller_output,
                                                           expected_output)
            relative_deltas = cls._compute_relative_deltas(absolute_deltas,
                                                           expected_output)
            root_mean_square_error = cls._rmse(controller_output,
                                               expected_output)
            print "Max absolute error: " + str(
                np.amax(absolute_deltas)) + "\nMax relative " \
                                            "error: " + str(
                np.amax(relative_deltas)) + ".\nRMSE: " + str(
                    root_mean_square_error)
            # TODO at this point what we need to do is to backpropagate the
            # error per neuron. Each absolute error value should be converted
            #  into Hz (frequency) values, for each neuron in output layer
            # should be identified the neurons that fired at the same time (
            # more or less - tbd) as the output layer neuron. Afterwards,
            # every synapse, that was used for simultaneously fired
            # neurons, should be adjusted accordingly (not clear how to
            # measure to which value the adjustment should happen).
            INSTANCE.increase_trainings_counter()
        else:
            print str(
                INSTANCE.get_calls_counter()) + ": network will not be trained"

    def get_current_behavior(self):
        return self._current_behavior

    def _sim_int_cb(self, msg):
        self._current_behavior = msg.current_behavior

    def increase_calls_counter(self):
        self._calls_counter += 1

    def increase_trainings_counter(self):
        self._trainings_counter += 1

    def get_calls_counter(self):
        return self._calls_counter

    def get_trainings_counter(self):
        return self._trainings_counter

    @classmethod
    def _validate_output(cls, output):
        for i in xrange(len(output)):
            value = output[i]
            if value > INSTANCE._atlas_joints_info_provider\
                    .get_max_value_for_joint(i):
                ratio = value / \
                        INSTANCE._atlas_joints_info_provider.get_max_value_for_joint(i)
                print "output value for joint " + str(i) + " exceeds it's " \
                                                           "maximum allowed " \
                                                           "value by " + str(
                        ratio)
            if value < INSTANCE._atlas_joints_info_provider\
                    .get_min_value_for_joint(i):
                ratio = INSTANCE._atlas_joints_info_provider\
                    .get_min_value_for_joint(i) / value
                print "output value for joint " + str(i) + " is lower than " \
                                                           "it's " \
                                                           "minimum allowed " \
                                                           "value by " + str(
                ratio)

    @classmethod
    def _compute_absolute_deltas(cls, controller_output, expected_output):
        expected_output_list = list(expected_output)
        result = []
        for i in xrange(len(controller_output)):
            delta = abs(expected_output_list[i] - controller_output[i])
            result.append(delta)
        return result

    @classmethod
    def _compute_relative_deltas(cls, absolute_deltas, expected_output):
        expected_output_list = list(expected_output)
        result = []
        for i in xrange(len(expected_output_list)):
            delta = absolute_deltas[i] / expected_output_list[i]
            result.append(delta)
        return result

    @classmethod
    def _rmse(cls, controller_output, expected_output):
        expected_output_list = list(expected_output)
        return np.sqrt(((np.asarray(expected_output_list) -
                         np.asarray(controller_output)) ** 2).mean())


INSTANCE = AtlasControllerTrainer()

def main():
    pass

if __name__ == '__main__':
    main()
