#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
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
        rospy.Subscriber('/turtle1/pose', Pose, self.store_collisions)
        pub1 = rospy.Publisher('/turtle_follower_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/turtle_follower_2/cmd_vel', Twist, queue_size=10)

        # Collision properties
        self._collision_pub = rospy.Publisher(
            '/turtle1/collision',
            String,
            queue_size=10
        )
        self._collisions = 0
        self._hit_the_wall = False

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
    rospy.init_node('followers')
    try:
        Follower()
    except rospy.ROSInterruptException:
        print ("end of the program")
