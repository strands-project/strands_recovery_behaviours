import rospy
import smach

from monitor_pause import MonitorPause
from monitored_navigation.recover_state_machine import RecoverStateMachine


class PauseState(RecoverStateMachine):
    def __init__(self):
        
        self.nav_stat=None
        
        RecoverStateMachine.__init__(self)
        self.resume_monitor=MonitorPause(is_paused=True)
        
        with self:
            smach.StateMachine.add('PAD_PAUSE_STATE',
                                   self.resume_monitor,
                                   transitions={'preempted':'preempted',
                                                'invalid':'recovered_without_help',
                                                'valid':'recovered_without_help'})

