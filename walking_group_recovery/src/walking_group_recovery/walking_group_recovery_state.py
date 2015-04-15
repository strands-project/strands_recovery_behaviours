import rospy
import smach

from monitored_navigation.recover_state import RecoverState

from os.path import join, exists, expanduser
from os import makedirs

from std_srvs.srv import Empty
from scitos_msgs.srv import EnableMotors
from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

import pygame

from pygame_managed_player.pygame_player import PyGamePlayer
from mongodb_media_server import MediaClient
from sound_player_server.srv import PlaySoundService, PlaySoundServiceRequest

from random import randint


class WalkingGroupRecovery(RecoverState):
    def __init__(self, name="walking_group_help", is_active=True, max_recovery_attempts=float("inf"), wait_for_nav_help_timeout=40, wait_for_nav_help_finished=60):
        RecoverState.__init__(self,
                             name=name,
                             outcomes=['recovered_with_help', 'recovered_without_help','not_recovered_with_help', 'not_recovered_without_help', 'preempted'],
                             input_keys=['goal','n_fails'],
                             output_keys=['goal','n_fails'],
                             is_active=is_active,
                             max_recovery_attempts=max_recovery_attempts
                             )

        rospy.set_param('wait_for_nav_help_timeout', wait_for_nav_help_timeout)
        rospy.set_param('wait_for_nav_help_finished', wait_for_nav_help_finished)

        self.enable_motors= rospy.ServiceProxy('enable_motors', EnableMotors)

        self.being_helped=True
        self.help_finished=False

        self.ask_help_srv=rospy.ServiceProxy('/monitored_navigation/human_help', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component='navigation'

        self.help_offered_service_name='walking_help_offered'
        self.help_offered_monitor=rospy.Service('/monitored_navigation/'+self.help_offered_service_name, Empty, self.help_offered_cb)

        self.help_finished_service_name="walking_help_finished"
        self.help_done_monitor=rospy.Service('/monitored_navigation/'+self.help_finished_service_name, Empty, self.help_finished_cb)

        self.sound_player_name = rospy.get_param("/walking_group_help/sound_player_server", "sound_player_server")
        self.music_file = rospy.get_param("/walking_group_help/music_file", "nooo.mp3")

        self.sound_player_server = rospy.ServiceProxy(self.sound_player_name, PlaySoundService)

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

    def active_execute(self, userdata):
        wait_for_nav_help_timeout=rospy.get_param('wait_for_nav_help_timeout',40)
        wait_for_nav_help_finished=rospy.get_param('wait_for_nav_help_finished',60)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self.service_msg.n_fails=self.n_tries
        self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
        self.service_msg.interaction_service=self.help_offered_service_name
        self.ask_help()

        for i in range(0,wait_for_nav_help_timeout):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if self.being_helped:
                break
            rospy.sleep(1)
        #pygame.mixer.init()
        #paths = roslib.packages.find_resource('walking_group_recovery', 'good_bad_ugly.mp3')
        #pygame.mixer.music.load(paths[0])
        #pygame.mixer.music.play()
        req = PlaySoundServiceRequest(self.music_file)
        resp = self.sound_player_server(req)
        print ("Played sound: " + str(resp.file_found) + " with priority " + str(resp.audio_priority))
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
                    self.service_preempt()
                    return 'preempted'


        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self.was_helped=self.being_helped or self.help_finished
        self.finish_execution()
        if self.was_helped:
            return 'recovered_with_help'
        else:
            return 'recovered_without_help'


    def finish_execution(self):
        #pygame.mixer.music.stop()
        self.enable_motors(True)
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.service_msg.interaction_service='none'
        self.ask_help()
        self.being_helped=False
        self.help_finished=False


    def service_preempt(self):
        self.finish_execution()
        smach.State.service_preempt(self)
