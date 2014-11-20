import rospy

import smach
from scitos_msgs.msg import BarrierStatus
from scitos_msgs.srv import ResetMotorStop
from monitored_navigation.recover_state_machine import RecoverStateMachine

from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

from mongo_logger import MonitoredNavEventClass




class RecoverMagneticStrip(RecoverStateMachine):
    def __init__(self,max_bumper_recovery_attempts=5):
        RecoverStateMachine.__init__(self)
        self.magnetic_state=MagneticState()
        
        with self:
            smach.StateMachine.add('MAGNETIC_STATE',
                                   self.magnetic_state,
                                   transitions={'help_done':'recovered_with_help',
                                                'preempted':'preempted'})


class MagneticState(smach.State):
    def __init__(self):
            smach.State.__init__(self,
                                outcomes=['help_done', 'preempted']
                                )
            
            self.barrier_stopped=True
            self.barrier_listener=rospy.Subscriber('/barrier_status', BarrierStatus, self.barrier_cb)
            
            self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)
            
            self.nav_stat=None
   
            self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help/', AskHelp)
            self.service_msg=AskHelpRequest()
            self.service_msg.failed_component='magnetic_strip'

            
            
    def barrier_cb(self, msg):
        self.barrier_stopped=msg.barrier_stopped        

  
   
    def ask_help(self):
        try:
            self.ask_help_srv(self.service_msg)
        except rospy.ServiceException, e:
            rospy.logwarn("No means of asking for human help available.")


    def execute(self, userdata):
        self.barrier_stopped=True

        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="magnetic_strip")
        
        i=1
        while self.barrier_stopped:
            self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
            self.ask_help()
            for j in range(0,120*i):
                rospy.sleep(1)
                if self.preempt_requested(): 
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
                    self.ask_help()
                    self.nav_stat.finalize(was_helped=False, n_tries=1)
                    self.nav_stat.insert()
                    self.service_preempt()
                    return 'preempted'
                if not self.barrier_stopped:
                    break
            i=i+1
        
        
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.ask_help()
        self.nav_stat.finalize(was_helped=True, n_tries=1)
        self.nav_stat.insert()
        self.reset_motorstop()
        if self.preempt_requested(): 
            return 'preempted'
        return "help_done"
      

    


