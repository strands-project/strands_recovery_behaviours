import rospy

import smach

from monitored_navigation.recover_state_machine import RecoverStateMachine
from recover_nav_backtrack import RecoverNavBacktrack


class RecoverNavNoHelp(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self,input_keys=['goal','n_nav_fails'],output_keys=['goal','n_nav_fails'])
        
        #self.register_start_cb(self.start_cb)
   
        self.backtrack=RecoverNavBacktrack()   
        with self:
            smach.StateMachine.add('BACKTRACK',
                                   self.backtrack,
                                   transitions={'succeeded':'recovered_without_help',
                                                'failure':'not_recovered_without_help',
                                                'preempted':'preempted'})
        

    