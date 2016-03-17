from __future__ import division
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState
import rospy

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
    def train_the_network(cls,controller):
        global INSTANCE
        INSTANCE.increase_calls_counter()
        if INSTANCE.get_current_behavior() == AtlasSimInterfaceCommand.WALK:
            print str(INSTANCE.get_trainings_counter())+": network is being " \
                                                     "trained"
            controller_output = controller.get_output()
            print "validating controller output"
            cls._validate_output(controller_output)
            expected_output = controller.get_current_state().position
            print "validating expected output"
            cls._validate_output(expected_output)
            INSTANCE.increase_trainings_counter()
        else:
            print str(INSTANCE.get_calls_counter())+": network will not be trained"

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


INSTANCE = AtlasControllerTrainer()

def main():
    pass

if __name__ == '__main__':
    main()
