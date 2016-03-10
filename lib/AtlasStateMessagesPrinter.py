import rospy
from atlas_msgs.msg import AtlasState, AtlasSimInterfaceState
import time
import numpy as np

global_message_content = np.zeros(28)
counter = 0
initialized = False

def state_cb(msg):
    global global_message_content
    global counter
    global initialized
    counter += 1
    if counter == 100:
        counter = 0
    else:
        return
    # print msg.position[AtlasState.back_lbz]
    message_content = msg.position
    if not initialized:
        global_message_content = message_content
        initialized = True
    av_deviation = np.zeros(len(message_content))
    for i in xrange(len(message_content)):
        av_deviation[i] = (global_message_content[i] - message_content[i]) ** 2
        if av_deviation[i] < 0.01:
            av_deviation[i] = 0
    # final_error = 0
    # for i in xrange(len(av_deviation)):
    #     final_error = final_error + av_deviation[i]
    # global_message_content = message_content
    # print final_error**(0.5)
    print zip(message_content,av_deviation)

def sim_int_cb(msg):
    global counter
    counter += 1
    if counter == 100:
        counter = 0
    else:
        return
    print msg.f_out[4]

def uhz_cb(msg):
    global global_message_content
    global counter
    global initialized
    left_ind = AtlasState.l_leg_mhx
    right_ind = AtlasState.r_leg_mhx
    counter += 1
    if counter == 100:
        counter = 0
    else:
        return
    message_content = msg.position
    if not initialized:
        global_message_content = message_content
        initialized = True
    av_deviation = np.zeros(len(message_content))
    i = left_ind
    av_deviation[i] = (global_message_content[i] - message_content[i]) ** 2
    if av_deviation[i] < 0.01:
            av_deviation[i] = 0
    i = right_ind
    av_deviation[i] = (global_message_content[i] - message_content[i]) ** 2
    if av_deviation[i] < 0.01:
            av_deviation[i] = 0
    print str(message_content[left_ind]) + ' : ' + str(
            av_deviation[left_ind]) + '; ' + str(
            message_content[right_ind]) + ' : ' + str(av_deviation[right_ind])

def main():
    rospy.init_node('atlas_state_messages_printer')
    subscriber = rospy.Subscriber('atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            sim_int_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
    print "stopped"