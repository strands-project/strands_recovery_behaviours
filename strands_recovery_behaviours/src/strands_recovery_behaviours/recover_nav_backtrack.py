import rospy

import smach

import actionlib
from actionlib_msgs.msg import GoalStatus
from backtrack_behaviour.msg import BacktrackAction, BacktrackGoal


#TODO add logging

class RecoverNavBacktrack(smach.State):
    def __init__(self, max_backtrack_attempts=0, backtrack_meters_back=0.8):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        rospy.set_param('max_backtrack_attempts', max_backtrack_attempts)   
        rospy.set_param('backtrack_meters_back', backtrack_meters_back)
        self.backtrack_client = actionlib.SimpleActionClient('/do_backtrack', BacktrackAction)
        rospy.loginfo("Waiting for backtrack action...") 
        self.backtrack_client.wait_for_server()
        rospy.loginfo("Done") 
        

    def execute(self, userdata):
        
        max_backtrack_attempts=rospy.get_param('max_backtrack_attempts',2)     
        backtrack_meters_back=rospy.get_param('backtrack_meters_back',0.8)
        if userdata.n_nav_fails > max_backtrack_attempts:
            return 'failure'
        
        backtrack_goal = BacktrackGoal()
        backtrack_goal.meters_back = backtrack_meters_back
        self.backtrack_client.send_goal(backtrack_goal)
        status = self.backtrack_client.get_state()
        while status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
            status = self.backtrack_client.get_state()
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.backtrack_client.wait_for_result(rospy.Duration(0.2))
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failure'
    
    def service_preempt(self):
        self.backtrack_client.cancel_all_goals()
        smach.State.service_preempt(self)

