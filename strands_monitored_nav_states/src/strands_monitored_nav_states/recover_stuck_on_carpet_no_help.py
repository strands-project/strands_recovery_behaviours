import rospy
import smach
from monitored_navigation.recover_state_machine import RecoverStateMachine
from recover_carpet_state import CarpetState
from geometry_msgs.msg import Twist

class RecoverStuckOnCarpetNoHelp(RecoverStateMachine):
    def __init__(self,max_recovery_attempts=float("inf")):
        RecoverStateMachine.__init__(self,input_keys=['n_fails'],output_keys=['n_fails'])
        self.state=CarpetState(max_recovery_attempts=max_recovery_attempts)
        
        with self:
            smach.StateMachine.add('CARPET_STATE',
                                   self.state,
                                   transitions={'preempted':'preempted',
                                                'recovered_without_help':'recovered_without_help',
                                                'preempted':'preempted',
                                                'not_active':'not_recovered_without_help'})
   

