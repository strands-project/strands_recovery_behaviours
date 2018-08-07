import rospy
import smach
from monitored_navigation.recover_state_machine import RecoverStateMachine
from recover_carpet_state import CarpetState
from recover_nav_states import Backtrack, Help
from geometry_msgs.msg import Twist

class RecoverStuckOnCarpetBacktrackHelp(RecoverStateMachine):
    def __init__(self,):
        RecoverStateMachine.__init__(self,input_keys=['n_fails'],output_keys=['n_fails'])
        self.state=CarpetState(max_recovery_attempts=10)
        self.backtrack=Backtrack(name='carpet_backtrack', max_recovery_attempts=12)   
        self.carpet_help=Help("carpet_help")
        with self:
            smach.StateMachine.add('CARPET_STATE',
                                   self.state,
                                   transitions={'preempted':'preempted',
                                                'recovered_without_help':'recovered_without_help',
                                                'not_active':'BACKTRACK'})
            smach.StateMachine.add('BACKTRACK',
                                   self.backtrack,
                                   transitions={'succeeded':'recovered_without_help',
                                                'failure':'CARPET_HELP',
                                                'not_active':'CARPET_HELP',
                                                'preempted':'preempted'})
            smach.StateMachine.add('CARPET_HELP',
                                   self.carpet_help,
                                   transitions={'recovered_with_help':'recovered_with_help', 
                                                'recovered_without_help':'recovered_without_help',
                                                'not_recovered_with_help':'not_recovered_with_help', 
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'not_active':'not_recovered_without_help',
                                                'preempted':'preempted'})


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
   

