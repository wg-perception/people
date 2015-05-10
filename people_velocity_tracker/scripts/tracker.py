#!/usr/bin/python

import roslib; roslib.load_manifest('people_velocity_tracker')
import rospy
from geometry_msgs.msg import Point, Vector3
import math
from people_msgs.msg import PositionMeasurementArray, Person, People
from easy_markers.generator import MarkerGenerator, Marker
from kalman_filter import Kalman

def distance(leg1, leg2):
    return math.sqrt(math.pow(leg1.x - leg2.x, 2) +
                     math.pow(leg1.y - leg2.y, 2) +
                     math.pow(leg1.z - leg2.z, 2))

def average(leg1, leg2):
    return Point((leg1.x + leg2.x) / 2,
                 (leg1.y + leg2.y) / 2,
                 (leg1.z + leg2.z) / 2)

def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

def subtract(v1, v2):
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)

def scale(v, s):
    v.x *= s
    v.y *= s
    v.z *= s

def printv(v):
    print "%.2f %.2f %.2f"%(v.x, v.y, v.z),

gen = MarkerGenerator()
gen.type = Marker.ARROW
gen.ns = 'velocities'
gen.lifetime = .5


class PersonEstimate(object):
    def __init__(self, msg):
        self.pos = msg
        self.reliability = 0.1
        self.k = Kalman()

    def update(self, msg):
        last = self.pos
        self.pos = msg
        self.reliability = max(self.reliability, msg.reliability)

        ivel = subtract(self.pos.pos, last.pos)
        time = (self.pos.header.stamp - last.header.stamp).to_sec()
        scale(ivel, 1.0 / time)

        self.k.update([ivel.x, ivel.y, ivel.z])

    def age(self):
        return self.pos.header.stamp

    def id(self):
        return self.pos.object_id

    def velocity(self):
        k = self.k.values()
        if k == None:
            return Vector3()
        v = Vector3(k[0], k[1], k[2])
        return v

    def publish_markers(self, pub):
        gen.scale = [.1, .3, 0]
        gen.color = [1, 1, 1, 1]
        vel = self.velocity()
        #scale(vel, 15)
        m = gen.marker(points=[self.pos.pos, add(self.pos.pos, vel)])
        m.header = self.pos.header
        pub.publish(m)

    def get_person(self):
        p = Person()
        p.name = self.id()
        p.position = self.pos.pos
        p.velocity = self.velocity()
        p.reliability = self.reliability
        return self.pos.header.frame_id, p


class VelocityTracker(object):
    def __init__(self):
        self.people = {}
        self.TIMEOUT = rospy.Duration(rospy.get_param('~timeout', 1.0))
        self.sub = rospy.Subscriber('/people_tracker_measurements',
                                    PositionMeasurementArray,
                                    self.pm_cb)
        self.mpub = rospy.Publisher('/visualization_marker',
                                    Marker,
                                    queue_size=10)
        self.ppub = rospy.Publisher('/people',
                                    People,
                                    queue_size=10)

    def pm_cb(self, msg):
        for pm in msg.people:
            if pm.object_id in self.people:
                self.people[pm.object_id].update(pm)
            else:
                p = PersonEstimate(pm)
                self.people[pm.object_id] = p

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Remove People Older Than timeout param
            now = rospy.Time.now()
            for p in self.people.values():
                if now - p.age() > self.TIMEOUT:
                    del self.people[p.id()]
            self.publish()
            rate.sleep()

    def publish(self):
        gen.counter = 0
        pl = People()
        pl.header.frame_id = None

        for p in self.people.values():
            p.publish_markers(self.mpub)
            frame, person = p.get_person()
            pl.header.frame_id = frame
            pl.people.append(person)

        self.ppub.publish(pl)

rospy.init_node("people_velocity_tracker")
vt = VelocityTracker()
vt.spin()
