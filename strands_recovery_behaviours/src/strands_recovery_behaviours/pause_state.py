import rospy
import smach

from monitor_pause import MonitorPause
from monitored_navigation.recover_state_machine import RecoverStateMachine

#TODO: log pauses? 
#from mongo_logger import MonitoredNavEventClass 




class PauseState(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self)
        self.resume_monitor=MonitorPause(is_paused=True)
        
        with self:
            smach.StateMachine.add('PAUSE_STATE',
                                   self.resume_monitor,
                                   transitions={'preempted':'preempted',
                                                'invalid':'recovered_without_help',
                                                'valid':'recovered_without_help'})
 
