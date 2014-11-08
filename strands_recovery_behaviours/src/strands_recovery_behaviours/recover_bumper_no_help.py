import rospy

import smach
from restart_bumper import RestartBumper
from monitored_navigation.recover_state_machine import RecoverStateMachine



from mongo_logger import MonitoredNavEventClass




class RecoverBumperNoHelp(RecoverStateMachine):
    def __init__(self,max_bumper_recovery_attempts=5):
        RecoverStateMachine.__init__(self)
        self.restart_bumper=RestartBumper()
        self.bumper_output=BumperOutput(5)
        
        with self:
            smach.StateMachine.add('RESTART_BUMPER',
                                   self.restart_bumper,
                                   transitions={'preempted':'preempted',
                                                'restarted':'BUMPER_OUTPUT'})
            smach.StateMachine.add('BUMPER_OUTPUT',
                                   self.bumper_output,
                                   transitions={'try_restart':'RESTART_BUMPER',
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'preempted':'preempted'})
        

class BumperOutput(smach.State):
    def __init__(self,max_bumper_recovery_attempts=5):
            smach.State.__init__(self,
                                outcomes=['recovered_without_help', 'not_recovered_without_help', 'preempted', "try_restart"],
                                input_keys=['recovered']
                                )
                                
            rospy.set_param('max_bumper_recovery_attempts', max_bumper_recovery_attempts)
            self.n_tries=0
            self.nav_stat=None

    def execute(self, userdata):
        
        max_bumper_recovery_attempts=rospy.get_param('max_bumper_recovery_attempts',5)
        
        if self.n_tries==0:
            self.nav_stat=MonitoredNavEventClass()
            self.nav_stat.initialize(recovery_mechanism="bumper_recovery_no_help")
            
         
        
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'
            
        if userdata.recovered:
            self.finish_execution()
            return "recovered_without_help"
        elif self.n_tries>max_bumper_recovery_attempts:
            self.finish_execution()
            return "not_recovered_without_help"
        else:
            self.n_tries=self.n_tries+1
            return "try_restart"
            
    
    def finish_execution(self):
        self.nav_stat.finalize(False,self.n_tries)
        self.nav_stat.insert()
        self.n_tries=0
    
    def service_preempt(self):
        self.finish_execution()
        smach.State.service_preempt(self)
        
