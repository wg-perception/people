#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import *
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller]
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    cmd = LaserTrajCmd()
    controller   =    sys.argv[1]
    cmd.header   =    roslib.msg.Header(None, None, None)
    cmd.profile  = "blended_linear"
    #cmd.pos      = [1.0, .26, -.26, -.7,   -.7,   -.26,   .26,   1.0, 1.0]
    d = .025
    #cmd.time     = [0.0, 0.4,  1.0, 1.1, 1.1+d,  1.2+d, 1.8+d, 2.2+d, 2.2+2*d]
    
#    cmd.pos = [1.2,  -.7, 1.2]
#    cmd.time= [0.0, 1.8, 2.0125 + .3]

    cmd.pos = [1.15,  0.6, 1.150]
    cmd.time= [0.0, 0.45, 0.9]

#    cmd.pos = [1.15,  0.6, -0.4, 0.6, 1.15]
#    cmd.time= [0.0, 0.45, 0.50, 0.55, 1.0]

    cmd.max_rate =  10   # Ignores param
    cmd.max_accel=  30   # ignores param

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.pos])
    print '  Time: %s' % ','.join(['%.3f' % x for x in cmd.time])
    print '  MaxRate: %f' % cmd.max_rate
    print '  MaxAccel: %f' % cmd.max_accel

    rospy.wait_for_service(controller + '/set_traj_cmd')                                        
    s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
    resp = s.call(SetLaserTrajCmdRequest(cmd))

    print 'Command sent!'
    print '  Response: %f' % resp.start_time.to_sec()
