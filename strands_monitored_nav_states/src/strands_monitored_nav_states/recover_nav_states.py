import rospy
import smach

from mongo_logger import MonitoredNavEventClass

from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Pose, PoseStamped

import actionlib
from actionlib_msgs.msg import GoalStatus
from backtrack_behaviour.msg import BacktrackAction, BacktrackGoal

from std_srvs.srv import Empty
from scitos_msgs.srv import EnableMotors
from strands_navigation_msgs.srv import AskHelp, AskHelpRequest


class ClearCostmaps(smach.State):
    def __init__(self, max_standalone_clear_attempts=1, wait_for_clearance_time=5):
        smach.State.__init__(self,
                             outcomes=['try_nav', 'do_other_recovery', 'preempted'],
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        rospy.set_param('max_standalone_clear_attempts', max_standalone_clear_attempts)
        rospy.set_param('wait_for_clearance_time', wait_for_clearance_time)

        self.clear_costmaps=rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.path_finder=rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        self.nav_stat=None
        
        
    def execute(self, userdata):
        max_standalone_clear_attempts=rospy.get_param('max_standalone_clear_attempts', 1)
        wait_for_clearance_time=rospy.get_param('wait_for_clearance_time', 5)
        
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="nav_clear_costmap")
        
        for i in range(0,wait_for_clearance_time):
            if self.preempt_requested():
                self.service_preempt(userdata.n_nav_fails)
                return 'preempted'
            rospy.sleep(1)
        
        
        self.clear_costmaps()
        current_pose=rospy.wait_for_message('/robot_pose', Pose, timeout=10)
        plan=self.path_finder(GetPlanRequest(start=PoseStamped(pose=current_pose), goal=userdata.goal.target_pose, tolerance=0.3))
        
        
        if self.preempt_requested():
            self.service_preempt(userdata.n_nav_fails)
            return 'preempted'
            
        self.nav_stat.finalize(was_helped=False,n_tries=userdata.n_nav_fails)
        self.nav_stat.insert()
        
        if len(plan.plan.poses)==0 or userdata.n_nav_fails>max_standalone_clear_attempts:
            return 'do_other_recovery'
        else:
            return 'try_nav'
            
    def service_preempt(self, n_tries):
        self.nav_stat.finalize(was_helped=False,n_tries=n_tries)
        self.nav_stat.insert()
        smach.State.service_preempt(self)


class Backtrack(smach.State):
    def __init__(self, max_backtrack_attempts=3, backtrack_meters_back=0.8):
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
        
        max_backtrack_attempts=rospy.get_param('max_backtrack_attempts',3)     
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
                self.nav_stat.insert()
                self.service_preempt()
                return 'preempted'
            self.backtrack_client.wait_for_result(rospy.Duration(0.2))
        self.nav_stat.finalize(was_helped=False,n_tries=userdata.n_nav_fails)
        self.nav_stat.insert()
        if self.preempt_requested():
            self.nav_stat.finalize(was_helped=False,n_tries=userdata.n_nav_fails)
            self.nav_stat.insert()
            self.service_preempt()
            return 'preempted'
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failure'
    
    def service_preempt(self):
        self.backtrack_client.cancel_all_goals()
        smach.State.service_preempt(self)
        
        
   
class Help(smach.State):
    def __init__(self, wait_for_nav_help_timeout=40, wait_for_nav_help_finished=60):
        smach.State.__init__(self,
                             # we need the number of move_base fails as
                             # incoming data from the move_base action state,
                             # because it is not possible for this recovery
                             # behaviour to check if it was succeeded
                             outcomes=['recovered_with_help', 'recovered_without_help','not_recovered_with_help', 'not_recovered_without_help', 'preempted'],   
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        rospy.set_param('wait_for_nav_help_timeout', wait_for_nav_help_timeout)
        rospy.set_param('wait_for_nav_help_finished', wait_for_nav_help_finished)
        self.nav_stat=None
            
        self.enable_motors= rospy.ServiceProxy('enable_motors', EnableMotors)
               
        self.being_helped=False
        self.help_finished=False
            
        self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help', AskHelp)
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
        
        wait_for_nav_help_timeout=rospy.get_param('wait_for_nav_help_timeout',40)
        wait_for_nav_help_finished=rospy.get_param('wait_for_nav_help_finished',60)
        
        
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="nav_help_recovery")
            
        if self.preempt_requested(): 
            self.service_preempt(userdata.n_nav_fails)
            return 'preempted'


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
                
    
        if self.preempt_requested():
            self.service_preempt(userdata.n_nav_fails)
            return 'preempted'  

        if self.being_helped or self.help_finished:
            self.finish_execution(userdata.n_nav_fails)
            return 'recovered_with_help'
        else:
            self.finish_execution(userdata.n_nav_fails)
            return 'recovered_without_help'



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
            

 

