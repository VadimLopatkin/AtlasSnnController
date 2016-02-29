import rospy
import time

from atlas_msgs.msg import AtlasState

class AtlasController():

    def __init__(self):
        rospy.init_node('atlas_controller')
        self.subscriber = rospy.Subscriber('atlas/atlas_state', AtlasState,
                                  self._state_cb)
        # rospy.spin()

    def _state_cb(self, msg):
        self._state = msg

    def get_state(self):
        return self._state
