import rospy

import smach

import actionlib
from actionlib_msgs.msg import GoalStatus
from backtrack_behaviour.msg import BacktrackAction, BacktrackGoal

from mongo_logger import MonitoredNavEventClass



class RecoverNavBacktrack(smach.State):
    def __init__(self, max_backtrack_attempts=2, backtrack_meters_back=0.8):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )
                             
        self.nav_stat=None

        rospy.set_param('max_backtrack_attempts', max_backtrack_attempts)   
        rospy.set_param('backtrack_meters_back', backtrack_meters_back)
        self.backtrack_client = actionlib.SimpleActionClient('/do_backtrack', BacktrackAction)
        got_server=self.backtrack_client.wait_for_server(rospy.Duration(1))
        while not got_server:   
            rospy.loginfo("Backtrack recovery state is waiting for backtrack action to start...")
            got_server=self.backtrack_client.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        rospy.loginfo("Backtrack recovery state initialized") 
        

    def execute(self, userdata):
        
        max_backtrack_attempts=rospy.get_param('max_backtrack_attempts',2)     
        backtrack_meters_back=rospy.get_param('backtrack_meters_back',0.8)
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if userdata.n_nav_fails > max_backtrack_attempts:
            return 'failure'
            
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="nav_backtrack")
            
        backtrack_goal = BacktrackGoal()
        backtrack_goal.meters_back = backtrack_meters_back
        self.backtrack_client.send_goal(backtrack_goal)
        status = self.backtrack_client.get_state()
        while status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
            status = self.backtrack_client.get_state()
            if self.preempt_requested():
                self.nav_stat.finalize(was_helped=False,n_tries=userdata.n_nav_fails)
                self.service_preempt()
                return 'preempted'
            self.backtrack_client.wait_for_result(rospy.Duration(0.2))
        self.nav_stat.finalize(was_helped=False,n_tries=userdata.n_nav_fails)
        self.nav_stat.insert()
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failure'
    
    def service_preempt(self):
        self.backtrack_client.cancel_all_goals()
        smach.State.service_preempt(self)

