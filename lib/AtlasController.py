import rospy
import time

from atlas_msgs.msg import AtlasState

from lib.SpikingNeuralNetwork import SpikingNeuralNetwork


class AtlasController():

    def __init__(self):
        rospy.init_node('atlas_controller')
        self.subscriber = rospy.Subscriber('atlas/atlas_state', AtlasState,
                                  self._state_cb)
        # rospy.spin()
        self._network = SpikingNeuralNetwork()

    def _state_cb(self, msg):
        self._state = msg
        self._network.process_input(self._state)

    def get_state(self):
        return self._state

    def get_network(self):
        return self._network
