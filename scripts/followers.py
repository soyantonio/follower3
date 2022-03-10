#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn


class Follower:
    def __init__(self):
        self._queue1 = []
        self._queue2 = []

        rospy.wait_for_service('spawn')
        try:
            spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
            #  rosservice type /spawn |  rossrv show
            spawn_turtle(5.5, 5.5, 0, "turtle_follower_1")
            spawn_turtle(5.5, 5.5, 0, "turtle_follower_2")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        rospy.Subscriber('/turtle1/cmd_vel', Twist, self.store_commands)
        pub1 = rospy.Publisher('/turtle_follower_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/turtle_follower_2/cmd_vel', Twist, queue_size=10)

        # Propagate messages
        rate = rospy.Rate(20)  # execute every 2 seconds
        while not rospy.is_shutdown():
            if len(self._queue2) > 0:
                cmd_2 = self._queue2.pop()
                pub2.publish(cmd_2)
            if len(self._queue1) > 0:
                cmd_1 = self._queue1.pop()
                pub1.publish(cmd_1)
                self._queue2.append(cmd_1)
            rate.sleep()

    def store_commands(self, data):
        """
        @param data: incoming data.
        @type  data: Twist
        """
        self._queue1.append(data)


if __name__ == '__main__':
    rospy.init_node('followers')
    try:
        Follower()
    except rospy.ROSInterruptException:
        print ("end of the program")
