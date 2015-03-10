#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import *
import actionlib
from strands_navigation_msgs.srv import *
from walking_group_recovery.srv import *

class ToggleWalkingGroupRecoveryServer(object):
    def __init__(self):
        rospy.init_node('toggle_walking_group_recovery_server')
        self.service = rospy.Service('toggle_walking_group_recovery', ToggleWalkingGroupRecovery, self.change_recovery)

    def change_recovery(self, req):
        s = rospy.ServiceProxy('/monitored_navigation/set_nav_recovery', SetNavRecovery)
        if req.use_walking_recovery:
            package = 'walking_group_recovery'
            recovery_file = 'walking_group_nav_states'
            recovery_class = 'WalkingGroupNav'
        else:
            package = 'strands_monitored_nav_states'
            recovery_file = 'recover_nav'
            recovery_class = 'RecoverNav'
        s(package, recovery_file, recovery_class)
        return ToggleWalkingGroupRecoveryResponse(package, recovery_file, recovery_class)
        
        

if __name__ == '__main__':
    server = ToggleWalkingGroupRecoveryServer()
    rospy.spin()

