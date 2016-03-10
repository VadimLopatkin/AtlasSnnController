import rospy
from atlas_msgs.msg import AtlasSimInterfaceState, AtlasState, AtlasCommand, \
    AtlasSimInterfaceCommand


class AtlasMessengerRecorder():
    def __init__(self):
        self._position_record = []
        self._current_behavior = 0
        self._sim_int_counter = 0
        self._global_record_counter = 0
        self._node = rospy.init_node('atlas_messenger_recorder')
        self._sim_interface_subscriber = rospy.Subscriber(
                'atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            self._sim_int_cb)
        self._atlas_state_suscriber = rospy.Subscriber(
                'atlas/atlas_state', AtlasState,
                                            self._atlas_state_cb)
        self._atlas_command_publisher = rospy.Publisher('/atlas/atlas_command',
                                                        AtlasCommand,
                                                        queue_size=1)
        self._atlas_sim_interface_command_publisher = rospy.Publisher(
                '/atlas/atlas_sim_interface_command',
                                                        AtlasSimInterfaceCommand,
                                                        queue_size=1)

    def _atlas_state_cb(self, msg):
        self._current_position = msg.position
        if self._current_behavior == 4:
            self._position_record.append(msg.position)
            print msg.position
        if (self._current_behavior == 3) and (len(self._position_record) > 0):
            # TODO do not forget to switch control mode
            self._proceed_with_walking()


    def _sim_int_cb(self, msg):
        self._sim_int_counter += 1
        if self._sim_int_counter == 100:
            self._sim_int_counter = 0
        else:
            return
        self._current_behavior = msg.current_behavior

    def get_position_record(self):
        return self._position_record

    def _proceed_with_walking(self):
        message = AtlasCommand()
        message.position = self._current_position
        if self._global_record_counter == len(self._position_record):
            self._global_record_counter = 0
        else:
            self._global_record_counter += 1
        for i in xrange(5,16):
            message.position[i] = self._position_record[
                self._global_record_counter][i]
        self._atlas_command_publisher.publish(message)


def main():
    recorder = AtlasMessengerRecorder()
    rospy.spin()
    print len(recorder.get_position_record())

if __name__ == '__main__':
    main()
    print "stopped"