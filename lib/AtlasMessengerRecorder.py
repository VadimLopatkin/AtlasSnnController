import rospy
import time
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
        if self._current_behavior == AtlasSimInterfaceCommand.WALK:
            self._position_record.append(msg.position)
            # print msg.position
        if (self._current_behavior == AtlasSimInterfaceCommand.STAND) and (len(
                self._position_record) > 0):
            self._switch_mode_to_manual()
            self._proceed_with_walking()
        if (self._current_behavior == AtlasSimInterfaceCommand.USER) and (
                    len(self._position_record) > 0):
            self._proceed_with_walking()
            time.sleep(0.02)


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
        position_list = list(self._current_position)
        if self._global_record_counter == len(self._position_record)-1:
            self._global_record_counter = 0
        else:
            self._global_record_counter += 1
        for i in xrange(5,16):
            position_list[i] = self._position_record[
                self._global_record_counter][i]
        message.position = tuple(position_list)
        message.k_effort = [255] * 28
        self._atlas_command_publisher.publish(message)
        time.sleep(0.02)

    def _switch_mode_to_manual(self):
        message = AtlasSimInterfaceCommand()
        message.header.stamp = rospy.Time.now()
        message.behavior = AtlasSimInterfaceCommand.USER
        self._atlas_sim_interface_command_publisher.publish(message)
        time.sleep(0.5)


def main():
    recorder = AtlasMessengerRecorder()
    rospy.spin()
    print len(recorder.get_position_record())

if __name__ == '__main__':
    main()
    print "stopped"