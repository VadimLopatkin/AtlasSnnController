import rospy

from lib.AtlasController import AtlasController

def main():
    print "start"
    controller = AtlasController(True)
    rospy.spin()
    print "finished"

if __name__ == '__main__':
    main()