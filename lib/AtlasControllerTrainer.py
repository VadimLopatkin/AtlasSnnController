from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState
import rospy


class AtlasControllerTrainer():
    def __init__(self):
        self._current_behavior = AtlasSimInterfaceCommand.NONE
        self._sim_interface_subscriber = rospy.Subscriber(
                'atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            self._sim_int_cb)

    @classmethod
    def train_the_network(cls,controller):
        global INSTANCE
        if INSTANCE.get_current_behavior() == AtlasSimInterfaceCommand.WALK:
            print "network is being trained"
        else:
            print "network will not be trained"

    def get_current_behavior(self):
        return self._current_behavior

    def _sim_int_cb(self, msg):
        self._current_behavior = msg.current_behavior

INSTANCE = AtlasControllerTrainer()

def main():
    pass

if __name__ == '__main__':
    main()
