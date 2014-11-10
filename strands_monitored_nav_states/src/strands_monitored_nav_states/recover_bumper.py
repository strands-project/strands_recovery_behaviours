import rospy

import smach
from restart_bumper import RestartBumper
from scitos_msgs.srv import EnableMotors
from monitored_navigation.recover_state_machine import RecoverStateMachine

from std_srvs.srv import Empty
from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

from mongo_logger import MonitoredNavEventClass




class RecoverBumper(RecoverStateMachine):
    def __init__(self,max_bumper_recovery_attempts=5):
        RecoverStateMachine.__init__(self)
        self.restart_bumper=RestartBumper()
        self.bumper_help=BumperHelp(5)
        
        with self:
            smach.StateMachine.add('RESTART_BUMPER',
                                   self.restart_bumper,
                                   transitions={'preempted':'preempted',
                                                'restarted':'BUMPER_HELP'})
            smach.StateMachine.add('BUMPER_HELP',
                                   self.bumper_help,
                                   transitions={'try_restart':'RESTART_BUMPER',
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'preempted':'preempted'})
 

class BumperHelp(smach.State):
    def __init__(self,max_bumper_recovery_attempts=5, wait_for_bumper_help_timeout=40, wait_for_bumper_help_finished=60):
            smach.State.__init__(self,
                                outcomes=['recovered_without_help', 'not_recovered_without_help', 'preempted', "try_restart"],
                                input_keys=[ 'recovered']
                                )
                                
            rospy.set_param('max_bumper_recovery_attempts', max_bumper_recovery_attempts)
            rospy.set_param('wait_for_bumper_help_timeout', wait_for_bumper_help_timeout)
            rospy.set_param('wait_for_bumper_help_finished', wait_for_bumper_help_finished)
            
            self.n_tries=0
            self.nav_stat=None
            
            self.enable_motors= rospy.ServiceProxy('enable_motors', EnableMotors)
            
            
            self.being_helped = False
            self.help_finished=False
            
            self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help/', AskHelp)
            self.service_msg=AskHelpRequest()
            self.service_msg.failed_component='bumper'
            
            
            self.help_offered_service_name='bumper_help_offered'
            self.help_offered_monitor=rospy.Service('/monitored_navigation/'+self.help_offered_service_name, Empty, self.help_offered_cb)
            
            self.help_finished_service_name="bumper_help_finished"
            self.help_done_monitor=rospy.Service('/monitored_navigation/'+self.help_finished_service_name, Empty, self.help_finished_cb)
            
            
            
    def help_offered_cb(self, req):
        self.being_helped=True
        return []
    
    def help_finished_cb(self, req):
        self.being_helped=False
        self.help_finished=True
        return []
        
       
   
    def ask_help(self):
        try:
            self.ask_help_srv(self.service_msg)
        except rospy.ServiceException, e:
            rospy.logwarn("No means of asking for human help available.")


    def execute(self, userdata):
        
        max_bumper_recovery_attempts=rospy.get_param('max_bumper_recovery_attempts',5)
        wait_for_bumper_help_timeout=rospy.get_param('wait_for_bumper_help_timeout',40)
        wait_for_bumper_help_finished=rospy.get_param('wait_for_bumper_help_finished',60)
        
        if self.n_tries==0:
            self.nav_stat=MonitoredNavEventClass()
            self.nav_stat.initialize(recovery_mechanism="bumper_recovery")
            
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'
        
        self.service_msg.n_tries=self.n_tries
        if userdata.recovered:
            self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
            self.service_msg.interaction_service='none'
            self.ask_help()
            if self.being_helped or self.help_finished:
                self.finish_execution()
                return "recovered_with_help"
            else:
                self.finish_execution()
                return "recovered_without_help"
        elif self.n_tries>max_bumper_recovery_attempts:
            self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
            self.service_msg.interaction_service='none'
            self.ask_help()
            if self.being_helped or self.help_finished:
                self.finish_execution()
                return "not_recovered_with_help"
            else:
                self.finish_execution()
                return "not_recovered_without_help"
        else:
            self.n_tries=self.n_tries+1
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if self.n_tries>1:
                self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                self.service_msg.interaction_service=self.help_offered_service_name
                self.ask_help()
            for i in range(0,wait_for_bumper_help_timeout):    
                if self.being_helped:
                    break
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                rospy.sleep(1)
            if self.being_helped:
                self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                self.service_msg.interaction_service=self.help_finished_service_name
                self.ask_help()
                for i in range(0,wait_for_bumper_help_finished):
                    self.enable_motors(False)
                    if self.help_finished:
                        break
                    rospy.sleep(1)
                self.being_helped=False
            return "try_restart"
                    
      
      
      
    def finish_execution(self):
        self.nav_stat.finalize(self.being_helped or self.help_finished,self.n_tries)
        self.being_helped=False
        self.help_finished=False
        self.nav_stat.insert()
        self.n_tries=0
    
    def service_preempt(self):
        self.finish_execution()
        self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
        self.service_msg.interaction_service='none'
        self.ask_help()
        smach.State.service_preempt(self)


