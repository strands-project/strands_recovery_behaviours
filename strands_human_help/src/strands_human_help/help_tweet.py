#! /usr/bin/env python

import rospy

from monitored_navigation.ui_helper import UIHelper

import actionlib
from strands_tweets.msg import GrabImageThenTweetAction, GrabImageThenTweetGoal

    
class HelpTweet(UIHelper):

    def __init__(self, bumper_fails_to_tweet=20, nav_fails_to_tweet=20):

        self.twitter=actionlib.SimpleActionClient('strands_image_tweets', GrabImageThenTweetAction)
        self.got_server=self.twitter.wait_for_server(rospy.Duration(1))
        countdown=10
        while not self.got_server and countdown>0:
            rospy.loginfo("Help via twitter is waiting for grab image then tweet action..." + str(countdown))
            self.got_server=self.twitter.wait_for_server(rospy.Duration(1))
            countdown=countdown-1
            if rospy.is_shutdown():
                return
        if self.got_server:
            rospy.loginfo("Help via twitter got grab image then tweet action action")
        else:
            rospy.logwarn("Timeout waiting for grab image then tweet action, no tweeting for help")
            
            
        rospy.set_param('bumper_fails_to_tweet',20)
        rospy.set_param('nav_fails_to_tweet',20)
        self.bumper_post="I've been crashed into something for quite some time now. Someone should come by and get me out of here."
        self.nav_post="I've been having some issue going around for quite some time now. I'm sure I'm pretty stuck somewhere. What does it look like?"
        self.magnetic_post="I'm very close to a flight of stairs! I'm frozen in terror! I need one of my handlers to come and help me!"
        self.tweet=GrabImageThenTweetGoal(topic='head_xtion/rgb/image_color')
        
        self.was_helped=False
        
        UIHelper.__init__(self)
        
        rospy.loginfo("Help via twitter initialized")
 

    def ask_help(self, failed_component, interaction_service, n_fails):
        if self.got_server:
            if failed_component=='navigation' and n_fails == rospy.get_param('nav_fails_to_tweet',20):
                self.tweet.text=self.nav_post
                self.twitter.send_goal_and_wait(self.tweet)
            elif failed_component=='bumper' and n_fails == rospy.get_param('bumper_fails_to_tweet',20):
                self.tweet.text=self.bumper_post
                self.twitter.send_goal_and_wait(self.tweet)
            elif failed_component=='magnetic_strip':
                self.tweet.text=self.magnetic_post
                self.twitter.send_goal_and_wait(self.tweet)
        
    def being_helped(self, failed_component, interaction_service, n_fails):
        return
        
    def help_finished(self, failed_component, interaction_service, n_fails):
        return
        
    def help_failed(self, failed_component, interaction_service, n_fails):
        return
    
   

 
  
