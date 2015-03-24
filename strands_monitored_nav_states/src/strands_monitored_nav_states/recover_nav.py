import rospy
import smach

from monitored_navigation.recover_state_machine import RecoverStateMachine
from recover_nav_states import  Backtrack, Help, SleepAndRetry

class RecoverNav(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self,input_keys=['goal','n_fails'],output_keys=['goal','n_fails'])
        self.sleep_and_retry=SleepAndRetry()
        self.backtrack=Backtrack()   
        self.nav_help=Help()
        with self:
            smach.StateMachine.add('SLEEP_AND_RETRY',
                                   self.sleep_and_retry,
                                   transitions={'preempted':'preempted',
                                                'try_nav':'recovered_without_help',
                                                'not_active':'BACKTRACK'})
            smach.StateMachine.add('BACKTRACK',
                                   self.backtrack,
                                   transitions={'succeeded':'recovered_without_help',
                                                'failure':'NAV_HELP',
                                                'not_active':'NAV_HELP',
                                                'preempted':'preempted'})
            smach.StateMachine.add('NAV_HELP',
                                   self.nav_help,
                                   transitions={'recovered_with_help':'recovered_with_help', 
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_with_help':'not_recovered_with_help', 
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'not_active':'not_recovered_without_help',
                                                'preempted':'preempted'})


