import rospy

import smach
from monitored_navigation.recover_state_machine import RecoverStateMachine
from strands_monitored_nav_states.recover_nav_states import  SleepAndRetry
from strands_monitored_nav_states.mongo_logger import MonitoredNavEventClass
from walking_group_recovery_state import WalkingGroupRecovery

class WalkingGroupNav(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self,input_keys=['goal','n_nav_fails'],output_keys=['goal','n_nav_fails'])
        self.sleep_and_retry=SleepAndRetry()
        self.walking_help = WalkingGroupRecovery()
        with self:
            #smach.StateMachine.add('SLEEP_AND_RETRY',
            #                       self.sleep_and_retry,
            #                       transitions={'preempted':'preempted',
            #                                    'try_nav':'recovered_without_help',
            #                                    'do_other_recovery':'WALKING_HELP'})
            smach.StateMachine.add('WALKING_HELP',
                                   self.walking_help,
                                   transitions={'recovered_with_help':'recovered_with_help', 
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_with_help':'not_recovered_with_help', 
                                                'not_recovered_without_help':'not_recovered_without_help', 
                                                'preempted':'preempted'})



