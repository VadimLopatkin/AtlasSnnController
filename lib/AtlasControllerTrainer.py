from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState
import rospy


class AtlasControllerTrainer:
    def __init__(self):
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
            print "network is being trained"
        else:
            print str(INSTANCE.get_calls_counter())+": network will not be trained"

    def get_current_behavior(self):
        return self._current_behavior

    def _sim_int_cb(self, msg):
        self._current_behavior = msg.current_behavior

    def increase_calls_counter(self):
        self._calls_counter += 1

    def get_calls_counter(self):
        return self._calls_counter


INSTANCE = AtlasControllerTrainer()

def main():
    pass

if __name__ == '__main__':
    main()
