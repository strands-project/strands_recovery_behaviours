import rospy

import smach
from scitos_msgs.msg import BarrierStatus
from scitos_msgs.srv import ResetMotorStop
from monitored_navigation.recover_state_machine import RecoverStateMachine
from monitored_navigation.recover_state import RecoverState
from strands_navigation_msgs.srv import AskHelp, AskHelpRequest


class RecoverMagneticStrip(RecoverStateMachine):
    def __init__(self,max_recovery_attempts=float("inf")):
        RecoverStateMachine.__init__(self)
        self.magnetic_state=MagneticState(max_recovery_attempts=max_recovery_attempts)
        
        with self:
            smach.StateMachine.add('MAGNETIC_STATE',
                                   self.magnetic_state,
                                   transitions={'help_done':'recovered_with_help',
                                                'preempted':'preempted',
                                                'not_active':'not_recovered_without_help'})


class MagneticState(RecoverState):
    def __init__(self,
                 name="recover_magnetic_strip",
                 is_active=True,
                 max_recovery_attempts=float("inf")):
                
        RecoverState.__init__(self,
                        name=name,
                        outcomes=['help_done', 'preempted'],
                        is_active=is_active,
                        max_recovery_attempts=max_recovery_attempts
                        )
        self.was_helped=False
        self.barrier_stopped=True
        self.barrier_listener=rospy.Subscriber('/barrier_status', BarrierStatus, self.barrier_cb)            
        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)              
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

    def active_execute(self, userdata):
        self.barrier_stopped=True
        i=1
        while self.barrier_stopped:
            self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
            self.ask_help()
            for j in range(0,120*i):
                rospy.sleep(1)
                if self.preempt_requested(): 
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
                    self.ask_help()
                    self.was_helped=False
                    self.service_preempt()
                    return 'preempted'
                if not self.barrier_stopped:
                    break
            i=i+1        
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.ask_help()
        self.was_helped=True
        self.reset_motorstop()
        if self.preempt_requested(): 
            return 'preempted'
        return "help_done"
      

    


