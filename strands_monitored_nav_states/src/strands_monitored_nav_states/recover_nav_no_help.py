import rospy
import smach

from monitored_navigation.recover_state_machine import RecoverStateMachine
from recover_nav_states import Backtrack, ClearCostmaps


class RecoverNavNoHelp(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self,input_keys=['goal','n_nav_fails'],output_keys=['goal','n_nav_fails'])
        
        self.clear_costmaps=ClearCostmaps()
        self.backtrack=Backtrack()   
        with self:
            smach.StateMachine.add('CLEAR_COSTMAPS',
                                   self.clear_costmaps,
                                   transitions={'preempted':'preempted',
                                                'try_nav':'recovered_without_help',
                                                'do_other_recovery':'BACKTRACK'})
            smach.StateMachine.add('BACKTRACK',
                                   self.backtrack,
                                   transitions={'succeeded':'recovered_without_help',
                                                'failure':'not_recovered_without_help',
                                                'preempted':'preempted'})
        

    