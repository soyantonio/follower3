#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose


class CollisionsCounter:
    def __init__(self):
        self._collisions = 0
        self._hit_the_wall = False
        self._collision_pub = rospy.Publisher('/turtle1/collision', String, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.store_collisions)
        rospy.spin()

    def store_collisions(self, data):
        """
        @param data: incoming data.
        @type  data: Pose
        """
        hit_left_wall = data.x < 0.5
        hit_right_wall = data.x > 10.9
        hit_top_wall = data.y > 10.9
        hit_bottom_wall = data.y < 0.5
        hit_wall = hit_left_wall or hit_right_wall or hit_top_wall or hit_bottom_wall

        if not self._hit_the_wall and hit_wall:
            self._collisions += 1

        self._hit_the_wall = hit_wall
        self._collision_pub.publish(str(self._collisions))


if __name__ == '__main__':
    rospy.init_node('collisions_counter')
    try:
        CollisionsCounter()
    except rospy.ROSInterruptException:
        print ("end of the program")
