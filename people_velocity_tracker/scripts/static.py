#!/usr/bin/python

import roslib; roslib.load_manifest('people_velocity_tracker')
import rospy
import sys
from people_msgs.msg import Person, People


class VelocityTracker(object):
    def __init__(self):
        self.ppub = rospy.Publisher('/people', People, queue_size=10)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pv = Person()
            pl = People()
            pl.header.stamp = rospy.Time.now()
            pl.header.frame_id = '/base_link'
            pv.position.x = float(sys.argv[1])
            pv.position.y = float(sys.argv[2])
            pv.position.z = .5
            pv.velocity.x = float(sys.argv[3])
            pv.velocity.y = float(sys.argv[4])
            pv.name = 'asdf'
            pv.reliability = .90
            pl.people.append(pv)

            self.ppub.publish(pl)
            rate.sleep()

rospy.init_node("people_velocity_tracker")
vt = VelocityTracker()
vt.spin()
