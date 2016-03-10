import rospy
from atlas_msgs.msg import AtlasSimInterfaceState, AtlasState


class AtlasMessengerRecorder():
    def __init__(self):
        self._position_record = []
        self._current_behavior = 0
        self._sim_int_counter = 0
        self._node = rospy.init_node('atlas_messenger_recorder')
        self._sim_interface_subscriber = rospy.Subscriber(
                'atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            self._sim_int_cb)
        self._atlas_state_suscriber = rospy.Subscriber(
                'atlas/atlas_state', AtlasState,
                                            self._atlas_state_cb)

    def _atlas_state_cb(self, msg):
        if(self._current_behavior == 4):
            self._position_record.append(msg.position)
            print msg.position


    def _sim_int_cb(self, msg):
        self._sim_int_counter += 1
        if self._sim_int_counter == 100:
            self._sim_int_counter = 0
        else:
            return
        self._current_behavior = msg.current_behavior

    def get_position_record(self):
        return self._position_record



def main():
    recorder = AtlasMessengerRecorder()
    rospy.spin()
    print len(recorder.get_position_record())

if __name__ == '__main__':
    main()
    print "stopped"