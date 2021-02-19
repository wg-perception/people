#!/usr/bin/python

import argparse

from people_msgs.msg import People, Person

import rospy


class VelocityTracker(object):
    def __init__(self, x, y, vx, vy):
        self.ppub = rospy.Publisher('/people', People, queue_size=10)
        self.person = Person()
        self.person.position.x = x
        self.person.position.y = y
        self.person.position.z = 0.5
        self.person.velocity.x = vx
        self.person.velocity.y = vy
        self.person.name = 'static_test_person'
        self.person.reliability = 0.90

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pl = People()
            pl.header.stamp = rospy.Time.now()
            pl.header.frame_id = '/base_link'
            pl.people.append(self.person)
            self.ppub.publish(pl)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('people_velocity_tracker')
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float, help='x coordinate')
    parser.add_argument('y', type=float, help='y coordinate')
    parser.add_argument('vx', type=float, nargs='?', default=0.0, help='x velocity')
    parser.add_argument('vy', type=float, nargs='?', default=0.0, help='y velocity')
    args = parser.parse_args()

    vt = VelocityTracker(args.x, args.y, args.vx, args.vy)
    vt.spin()
