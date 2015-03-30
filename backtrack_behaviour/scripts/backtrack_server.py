#! /usr/bin/env python

import rospy

from nav_msgs.msg import Path
from nav_msgs.srv import *
from move_base_msgs.msg import *
import dynamic_reconfigure.client
from scitos_ptu.msg import *
from backtrack_behaviour.srv import PreviousPosition
from backtrack_behaviour.srv import RepublishPointcloud
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from mary_tts.msg import maryttsAction, maryttsGoal

import actionlib

from backtrack_behaviour.msg import BacktrackAction, BacktrackResult, BacktrackFeedback

class BacktrackServer(object):
    def __init__(self):
        rospy.init_node('backtrack_actionserver')
        self.consider_moving_success = True

        # get ptu action client
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        rospy.loginfo("Waiting for ptu action...")
        self.ptu_action_client.wait_for_server()
        rospy.loginfo("Done")

        # get move_base action client
        self.move_base_action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client.wait_for_server()
        rospy.loginfo("Done")
        self.move_base_reconfig_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')

        # get planner service
        rospy.loginfo("Waiting for move_base planner service...")
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        self.planner = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

        # get mary_tts action client
        self.speaker = actionlib.SimpleActionClient('/speak', maryttsAction)
        got_server = self.speaker.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Backtrack behaviour is waiting for marytts action...")
            got_server = self.speaker.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return

        rospy.loginfo("Backtrack behaviour got marytts action")
        self.speech = "I am stuck here, so I might try moving backwards. Please get out of the way."

        # set correct parameters and announce server
        self.global_plan = None
        self.last_global_plan_time = None
        rospy.Subscriber("/move_base/NavfnROS/plan" , Path, self.global_planner_checker_cb)

        self.feedback = BacktrackFeedback()
        self.result = BacktrackResult()

        self.server = actionlib.SimpleActionServer('do_backtrack', BacktrackAction, self.execute, False)
        self.server.start()
        rospy.loginfo("/do_backtrack action server started")

    def global_planner_checker_cb(self, msg):
        self.global_plan = msg
        self.last_global_plan_time = rospy.get_rostime()

    def stop_republish(self):
        try:
            republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
            republish_pointcloud(False, '', '', 0.0)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't stop republish pointcloud service, returning failure.")
            return False
        return True

    def start_republish(self):
        try:
            republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
            republish_pointcloud(True, '/head_xtion/depth/points', '/move_base/head_points_clearing', 0.05)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't get republish pointcloud service, returning failure.")
            return False
        return True

    def pose_cb(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.pose = msg
        if self.first_pose:
            self.first_x = self.x
            self.first_y = self.y
            self.first_pose = False

    def gone_too_far(self, meters_back):
        xdiff = self.x - self.first_x
        ydiff = self.y - self.first_y
        if xdiff*xdiff + ydiff*ydiff > (meters_back+0.2)*(meters_back+0.2):
            return True
        return False

    def has_moved(self):
        xdiff = self.x - self.first_x
        ydiff = self.y - self.first_y
        if xdiff*xdiff + ydiff*ydiff > 0.2*0.2:
            return True

    def reset_ptu(self):
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 0
        ptu_goal.tilt = 0
        ptu_goal.pan_vel = 30
        ptu_goal.tilt_vel = 30
        self.ptu_action_client.send_goal(ptu_goal)
        #self.ptu_action_client.wait_for_result()

    def reset_move_base_pars(self):
        params = { 'max_vel_x' : self.max_vel_x, 'min_vel_x' : self.min_vel_x }
        config = self.move_base_reconfig_client.update_configuration(params)

    def execute(self, goal):
        #back track in elegant fashion
        sources = rospy.get_param("/move_base/local_costmap/obstacle_layer/observation_sources")
        if "head_cloud_sensor" not in sources:
            rospy.logwarn("Navigation with head camera not active in move_base configuration, aborting...")
            self.server.set_aborted()
            return # 'failure'
        try:
            previous_position = rospy.ServiceProxy('previous_position', PreviousPosition)
            meter_back = previous_position(goal.meters_back)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't get previous position service, returning failure.")
            self.server.set_aborted()
            return # 'failure'

        print "Got the previous position: ", meter_back.previous_pose.pose.position.x, ", ", meter_back.previous_pose.pose.position.y, ", ",  meter_back.previous_pose.pose.position.z

        self.move_base_action_client.cancel_all_goals() # begin by cancelling others
        self.first_pose = True
        pose_sub = rospy.Subscriber("/robot_pose" , Pose, self.pose_cb)

        self.feedback.status = "Moving the PTU"
        self.server.publish_feedback(self.feedback)
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = -175
        ptu_goal.tilt = 30
        ptu_goal.pan_vel = 30
        ptu_goal.tilt_vel = 30
        self.ptu_action_client.send_goal(ptu_goal)
        self.ptu_action_client.wait_for_result()

        if self.server.is_preempt_requested():
            self.reset_ptu()
            pose_sub.unregister()
            self.service_preempt()
            return

        if self.start_republish() == False:
            self.reset_ptu()
            pose_sub.unregister()
            self.server.set_aborted()
            return # 'failure'

        print "Managed to republish pointcloud."

        self.max_vel_x = rospy.get_param("/move_base/DWAPlannerROS/max_vel_x")
        self.min_vel_x = rospy.get_param("/move_base/DWAPlannerROS/min_vel_x")
        params = { 'max_vel_x' : 0.2, 'min_vel_x' : -0.2 }
        try:
            config = self.move_base_reconfig_client.update_configuration(params)
        except:
            rospy.logwarn("Could not reconfigure move_base parameters, returning failure.")
            self.reset_ptu()
            pose_sub.unregister()
            self.service_preempt()
            return

        self.feedback.status = "Moving the robot"
        self.server.publish_feedback(self.feedback)
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose = meter_back.previous_pose.pose
        move_goal.target_pose.header.frame_id = meter_back.previous_pose.header.frame_id
        #rospy.sleep(rospy.Duration.from_sec(1))
        #print movegoal

        if not self.first_pose:
            try:
                plan_request = GetPlanRequest()
                plan_request.start.pose = self.pose
                plan_request.start.header.frame_id = '/map'
                plan_request.goal = move_goal.target_pose
                resp = self.planner(plan_request)
                if len(resp.plan.poses) > 0:
                    self.speaker.send_goal(maryttsGoal(text=self.speech))
                    rospy.sleep(5)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

        self.move_base_action_client.send_goal(move_goal)
        status = self.move_base_action_client.get_state()
        #did_speak = False
        while status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
            status = self.move_base_action_client.get_state()
            if self.server.is_preempt_requested():
                self.stop_republish()
                self.reset_move_base_pars()
                self.reset_ptu()
                pose_sub.unregister()
                self.service_preempt()
                return
            if self.gone_too_far(goal.meters_back):
                self.stop_republish()
                self.reset_move_base_pars()
                self.reset_ptu()
                self.move_base_action_client.cancel_all_goals()
                pose_sub.unregister()
                self.server.set_aborted()
                return
            #if not did_speak:
            #     last_plan_recent = rospy.get_rostime() - self.last_global_plan_time < rospy.Duration.from_sec(1)
            #     last_plan_good = len(self.global_plan.poses) > 0
            #     if last_plan_recent and last_plan_good:
            #         self.speaker.send_goal(maryttsGoal(text=self.speech))
            #         did_speak = True
            #         #rospy.sleep(2)

            self.move_base_action_client.wait_for_result(rospy.Duration(0.2))

        self.reset_move_base_pars()
        pose_sub.unregister()

        if self.server.is_preempt_requested():
            self.stop_republish()
            self.service_preempt()
            return

        if self.stop_republish() == False:
            self.server.set_aborted()
            return # 'failure'

        self.reset_ptu()

        print "Reset PTU, move_base parameters and stopped republishing pointcloud."

        if self.server.is_preempt_requested():
            self.service_preempt()
            return

        if self.consider_moving_success and self.has_moved():
            self.result.status = 'failure_but_moved'
            self.server.set_succeeded(self.result)
            return
        elif self.move_base_action_client.get_state() == GoalStatus.SUCCEEDED: #set the previous goal again
            self.result.status = 'reached_point'
            self.server.set_succeeded(self.result)
            return
        elif status == GoalStatus.PREEMPTED:
            self.service_preempt()
            return
        else:
            if (rospy.get_rostime() - self.last_global_plan_time < rospy.Duration.from_sec(1)) and (self.global_plan.poses == []):
                self.server.set_aborted()
                return # 'failure' # 'global_plan_failure'
            else: # 'local_plan_failure'
                self.result.status = 'local_plan_failure' # 'succeeded'
                self.server.set_succeeded(self.result)
                return

    def service_preempt(self):
        #check if preemption is working
        self.move_base_action_client.cancel_all_goals()
        self.server.set_preempted()

if __name__ == '__main__':
    server = BacktrackServer()
    rospy.spin()
