import rospy
import time

from atlas_msgs.msg import AtlasState, AtlasCommand
from osrf_msgs.msg import JointCommands

from lib.SpikingNeuralNetwork import SpikingNeuralNetwork


class AtlasController():

    def __init__(self):
        rospy.init_node('atlas_controller')
        self._subscriber = rospy.Subscriber('atlas/atlas_state', AtlasState,
                                            self._state_cb)
        self._network = SpikingNeuralNetwork()

    def _state_cb(self, msg):
        self._state = msg
        self._network.process_input(self._state)

    def get_state(self):
        return self._state

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