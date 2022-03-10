#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn


class Follower:
    def __init__(self):
        rospy.wait_for_service('spawn')
        try:
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            #  rosservice type /spawn |  rossrv show
            spawn_turtle(5.5, 5.5, 0, "turtle_follower_1")
            spawn_turtle(5.5, 5.5, 0, "turtle_follower_2")

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('followers')
    try:
        Follower()
    except rospy.ROSInterruptException:
        print ("end of the program")
