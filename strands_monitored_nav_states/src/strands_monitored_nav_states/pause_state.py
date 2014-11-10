import rospy
import smach

from monitor_pause import MonitorPause
from monitored_navigation.recover_state_machine import RecoverStateMachine

from mongo_logger import MonitoredNavEventClass 




class PauseState(RecoverStateMachine):
    def __init__(self):
        
        self.nav_stat=None
        
        RecoverStateMachine.__init__(self)
        self.resume_monitor=MonitorPause(is_paused=True)
        
        with self:
            smach.StateMachine.add('PAUSE_STATE',
                                   self.resume_monitor,
                                   transitions={'preempted':'preempted',
                                                'invalid':'recovered_without_help',
                                                'valid':'recovered_without_help'})
                                                
    def execute(self, userdata):
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="paused")
        result=smach.StateMachine.execute(self, userdata)
        self.nav_stat.finalize(was_helped=False,n_tries=0)
        self.nav_stat.insert()
        return result
 
