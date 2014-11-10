import rospy

import smach
from std_srvs.srv import Empty
from scitos_msgs.srv import EnableMotors
from monitored_navigation.recover_state_machine import RecoverStateMachine

from recover_nav_backtrack import RecoverNavBacktrack

from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

from mongo_logger import MonitoredNavEventClass



class RecoverNav(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self,input_keys=['goal','n_nav_fails'],output_keys=['goal','n_nav_fails'])
        
   
        self.backtrack=RecoverNavBacktrack()   
        self.nav_help=RecoverNavHelp()
        with self:
            smach.StateMachine.add('BACKTRACK',
                                   self.backtrack,
                                   transitions={'succeeded':'recovered_without_help',
                                                'failure':'NAV_HELP',
                                                'preempted':'preempted'})
            smach.StateMachine.add('NAV_HELP',
                                   self.nav_help,
                                   transitions={'recovered_with_help':'recovered_with_help', 
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_with_help':'not_recovered_with_help', 
                                                'not_recovered_without_help':'not_recovered_without_help', 
                                                'preempted':'preempted'})


class RecoverNavHelp(smach.State):
    def __init__(self,max_nav_recovery_attempts=5, wait_for_nav_help_timeout=40, wait_for_nav_help_finished=60):
        smach.State.__init__(self,
                             # we need the number of move_base fails as
                             # incoming data from the move_base action state,
                             # because it is not possible for this recovery
                             # behaviour to check if it was succeeded
                             outcomes=['recovered_with_help', 'recovered_without_help','not_recovered_with_help', 'not_recovered_without_help', 'preempted'],   
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        rospy.set_param('max_nav_recovery_attempts', max_nav_recovery_attempts)
        rospy.set_param('wait_for_nav_help_timeout', wait_for_nav_help_timeout)
        rospy.set_param('wait_for_nav_help_finished', wait_for_nav_help_finished)
        self.nav_stat=None
            
        self.enable_motors= rospy.ServiceProxy('enable_motors', EnableMotors)
               
        self.being_helped=False
        self.help_finished=False
            
        self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help/manager', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component='navigation'
            
        self.help_offered_service_name='nav_help_offered'
        self.help_offered_monitor=rospy.Service('/monitored_navigation/'+self.help_offered_service_name, Empty, self.help_offered_cb)
            
        self.help_finished_service_name="nav_help_finished"
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
        
        max_nav_recovery_attempts=rospy.get_param('max_nav_recovery_attempts',5)
        wait_for_nav_help_timeout=rospy.get_param('wait_for_nav_help_timeout',40)
        wait_for_nav_help_finished=rospy.get_param('wait_for_nav_help_finished',60)
        
        
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="nav_help_recovery")
            
        if self.preempt_requested(): 
            self.service_preempt(userdata.n_nav_fails)
            return 'preempted'

        if userdata.n_nav_fails < max_nav_recovery_attempts:
   
            self.service_msg.n_fails=userdata.n_nav_fails
            self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
            self.service_msg.interaction_service=self.help_offered_service_name
            self.ask_help()

            for i in range(0,wait_for_nav_help_timeout):
                if self.preempt_requested():
                    self.service_preempt(userdata.n_nav_fails)
                    return 'preempted'
                if self.being_helped:
                    break
                rospy.sleep(1)                   
            if self.being_helped:
                self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                self.service_msg.interaction_service=self.help_finished_service_name
                self.ask_help()
                for i in range(0,wait_for_nav_help_finished):
                    self.enable_motors(False) 
                    if self.help_finished:
                        break
                    rospy.sleep(1)
                    
        
            if self.preempt_requested():
                self.service_preempt(userdata.n_nav_fails)
                return 'preempted'  

            if self.being_helped or self.help_finished:
                self.finish_execution(userdata.n_nav_fails)
                return 'recovered_with_help'
            else:
                self.finish_execution(userdata.n_nav_fails)
                return 'recovered_without_help'
        else:
            userdata.n_nav_fails=0
            if self.being_helped or self.help_finished:
                self.finish_execution(userdata.n_nav_fails)
                return 'not_recovered_with_help'
            else:
                self.finish_execution(userdata.n_nav_fails)
                return 'not_recovered_without_help'


    def finish_execution(self, n_tries):
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.service_msg.interaction_service='none'
        self.ask_help()
        self.nav_stat.finalize(self.being_helped or self.help_finished,n_tries)
        self.being_helped=False
        self.help_finished=False
        self.nav_stat.insert()
    
    
    def service_preempt(self, n_tries):
        self.finish_execution(n_tries)
        smach.State.service_preempt(self)
            

 