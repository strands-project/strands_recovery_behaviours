import rospy

import smach
from scitos_msgs.srv import EnableMotors, ResetMotorStop
from scitos_msgs.msg import MotorStatus
from monitored_navigation.recover_state_machine import RecoverStateMachine
from monitored_navigation.recover_state import RecoverState

from std_srvs.srv import Empty
from strands_navigation_msgs.srv import AskHelp, AskHelpRequest



class RecoverBumper(RecoverStateMachine):
    def __init__(self,max_recovery_attempts=float("inf")):
        RecoverStateMachine.__init__(self)
        self.bumper_help=BumperHelp(max_recovery_attempts=max_recovery_attempts)

        with self:
            smach.StateMachine.add('BUMPER_HELP',
                                   self.bumper_help,
                                   transitions={'recovered_without_help':'recovered_without_help',
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'recovered_with_help':'recovered_with_help',
                                                'not_recovered_with_help':'not_recovered_with_help',
                                                'preempted':'preempted',
                                                'not_active':'not_recovered_without_help'})

class BumperHelp(RecoverState):
    def __init__(self,
                 name="recover_bumper",
                 is_active=True,
                 max_recovery_attempts=float("inf"),
                 wait_for_bumper_help_timeout=40,
                 wait_for_bumper_help_finished=60):
        RecoverState.__init__(self,
                        name=name,
                        outcomes=['recovered_without_help', 'not_recovered_without_help','recovered_with_help', 'not_recovered_with_help', 'preempted'],
                        is_active=is_active,
                        max_recovery_attempts=max_recovery_attempts
                        )

        rospy.set_param('wait_for_bumper_help_timeout', wait_for_bumper_help_timeout)
        rospy.set_param('wait_for_bumper_help_finished', wait_for_bumper_help_finished)
        self.wait_reset_bumper_duration = rospy.get_param('~wait_reset_bumper_duration', 0.0)

        self.n_tries=1
        self.being_helped = False
        self.help_finished=False

        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)
        self.enable_motors= rospy.ServiceProxy('enable_motors', EnableMotors)
        self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help/', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component='bumper'
        self.help_offered_service_name='bumper_help_offered'
        self.help_offered_monitor=rospy.Service('/monitored_navigation/'+self.help_offered_service_name, Empty, self.help_offered_cb)
        self.help_finished_service_name="bumper_help_finished"
        self.help_done_monitor=rospy.Service('/monitored_navigation/'+self.help_finished_service_name, Empty, self.help_finished_cb)

        self.motor_monitor = rospy.Subscriber("/motor_status", MotorStatus, self.bumper_monitor_cb)
        self.bumper_pressed=True

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

    def bumper_monitor_cb(self, msg):
        self.bumper_pressed = msg.bumper_pressed

    def active_execute(self, userdata):
        wait_for_bumper_help_timeout=rospy.get_param('wait_for_bumper_help_timeout',40)
        wait_for_bumper_help_finished=rospy.get_param('wait_for_bumper_help_finished',60)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        self.service_msg.n_fails=self.n_tries
        while self.bumper_pressed:
            max_recovery_attempts=rospy.get_param("/monitored_navigation/recover_states/"+self.name, (True, float("inf")))[1]
            if self.n_tries>max_recovery_attempts:
                self.finish_execution()
                if self.was_helped:
                    return "not_recovered_with_help"
                else:
                    return "not_recovered_without_help"
            else:
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                if not self.help_finished:
                    self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                    self.service_msg.interaction_service=self.help_offered_service_name
                    self.ask_help()
                self.being_helped=False
                self.help_finished=False
                for i in range(0,wait_for_bumper_help_timeout):
                    if self.being_helped:
                        break
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    if not self.bumper_pressed:
                        return self.recover_succeeded()
                    rospy.sleep(1)
                if self.being_helped:
                    self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                    self.service_msg.interaction_service=self.help_finished_service_name
                    self.ask_help()
                    for i in range(0,wait_for_bumper_help_finished):
                        self.enable_motors(False)
                        if self.help_finished:
                            if self.bumper_pressed:
                                break
                            else:
                                return self.recover_succeeded()
                        rospy.sleep(1)
            self.n_tries=self.n_tries+1
            self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
            self.service_msg.interaction_service=self.help_offered_service_name
            self.ask_help()
            rospy.sleep(0.5)
        return self.recover_succeeded()


    def recover_succeeded(self):
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.service_msg.interaction_service='none'
        self.ask_help()
        self.finish_execution()
        if self.was_helped:
            return "recovered_with_help"
        else:
            return "recovered_without_help"

    def finish_execution(self):
        self.was_helped=self.being_helped or self.help_finished
        if not self.was_helped:
            rospy.sleep(self.wait_reset_bumper_duration)
        self.reset_motorstop()
        self.enable_motors(False)    
        self.enable_motors(True)
        self.reset_motorstop()
        self.being_helped=False
        self.help_finished=False

    def service_preempt(self):
        self.finish_execution()
        self.was_helped=False
        self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
        self.service_msg.interaction_service='none'
        self.ask_help()
        smach.State.service_preempt(self)
