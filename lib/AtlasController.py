import rospy

from atlas_msgs.msg import AtlasState, AtlasSimInterfaceState
from osrf_msgs.msg import JointCommands

from lib.SpikingNeuralNetwork import SpikingNeuralNetwork
from lib.AtlasControllerTrainer import AtlasControllerTrainer


class AtlasController():

    def __init__(self, learning_mode=False):
        self._initializing = True
        self._learning_mode = learning_mode
        if self._learning_mode:
            self._current_state = AtlasState()
        rospy.init_node('atlas_controller')
        self._subscriber = rospy.Subscriber('atlas/atlas_state', AtlasState,
                                            self._state_cb)
        self._network = SpikingNeuralNetwork()
        self._initializing = False

    def _state_cb(self, msg):
        if self._initializing:
            self._current_state = msg
            return
        if self._learning_mode:
            self._previous_state = self._current_state
            self._current_state = msg
            self._network.process_input(self._previous_state)
            AtlasControllerTrainer.train_the_network(self)
        else:
            self._current_state = msg
            self._network.process_input(self._current_state)

    def get_current_state(self):
        return self._current_state

    def get_network(self):
        return self._network


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