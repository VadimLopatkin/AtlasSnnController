import rospy
import time

rospy.init_node('atlas_controller_command_printer')
while True:
    param_name = 'atlas_controller/gains'
    value = rospy.get_param(param_name)
    print value
    time.sleep(0.02)