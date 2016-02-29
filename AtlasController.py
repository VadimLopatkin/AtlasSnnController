import rospy
import time

from atlas_msgs.msg import AtlasState

print rospy

def state_cb( msg):
    print msg
    time.sleep(0.5)

def main():
    rospy.init_node('atlas_controller')
    rospy.Subscriber('atlas/atlas_state', AtlasState, state_cb)
    rospy.spin()
    print "working"


if __name__ == '__main__':
    main()