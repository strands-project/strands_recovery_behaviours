#!/usr/bin/env python

import rospy
from strands_navigation_msgs.msg import DynClassLoaderDef
from strands_navigation_msgs.srv import SetNavRecovery
from walking_group_recovery.srv import *

class ToggleWalkingGroupRecoveryServer(object):
    def __init__(self):
        rospy.init_node('toggle_walking_group_recovery_server')
        #Not too nice but don't know a different way of using params in recovery behaviour
        rospy.set_param("/walking_group_help/sound_player_server", rospy.get_param("~sound_player_server", "walking_group_player"))
        rospy.set_param("/walking_group_help/music_file", rospy.get_param("~music_file", "nooo.mp3"))
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
        s(DynClassLoaderDef(package, recovery_file, recovery_class))
        return ToggleWalkingGroupRecoveryResponse(package, recovery_file, recovery_class)



if __name__ == '__main__':
    server = ToggleWalkingGroupRecoveryServer()
    rospy.spin()
