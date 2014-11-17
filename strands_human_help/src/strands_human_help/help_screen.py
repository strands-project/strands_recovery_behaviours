#! /usr/bin/env python

import rospy
import roslib

from monitored_navigation.ui_helper import UIHelper

import strands_webserver.client_utils
import strands_webserver.page_utils

 
class HelpScreen(UIHelper):

    def __init__(self, webserver_srv_prefix='strands_webserver', service_prefix='/monitored_navigation', set_http_root=False):
        
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service(webserver_srv_prefix + '/display_page', 1)
                rospy.wait_for_service(webserver_srv_prefix + '/get_hostname',1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("help via screen is waiting for webserver services...")
            if rospy.is_shutdown():
                return

        rospy.loginfo("help via screen got webserver services")   
        self.display_no = rospy.get_param("~display", 0)
        if set_http_root:
            strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('strands_human_help/html'))           
        self.display_main_page()
        
        
        self.help_label='HELP'
        self.finish_label='OK'
        self.bumper_help_html='Help! My bumper is stuck against something. If you can help me, please press the button below.'
        self.nav_help_html='Help! I appear to have trouble moving past an obstruction. If you can help me, please press the button below.' 
        self.help_instructions_html='Thank you! Please follow these instructions:' 
        self.help_instructions_html += '<hr/>' 
        self.help_instructions_html += '<ol>'
        self.help_instructions_html += '<li>Push me away from any obstructions into a clear space</li>'
        self.help_instructions_html += '<li>Press the '+ self.finish_label +' button below</li>'
        self.help_instructions_html += '</ol>'
    
        self.service_prefix=service_prefix
        UIHelper.__init__(self)
        
        rospy.loginfo("help via screen initialized")  

  
    def ask_help(self, failed_component, interaction_service, n_fails):
        if failed_component=='navigation':
            self.generate_help_content(self.help_label, self.nav_help_html,  interaction_service)
        elif failed_component=='bumper':
            self.generate_help_content(self.help_label, self.bumper_help_html,  interaction_service)
        
    def being_helped(self, failed_component, interaction_service, n_fails):
        self.generate_help_content(self.finish_label, self.help_instructions_html,  interaction_service)
        
    def help_finished(self, failed_component, interaction_service, n_fails):
        self.display_main_page()
        
    def help_failed(self, failed_component, interaction_service, n_fails):
        self.ask_help(failed_component, interaction_service, n_fails)
    
    
    def display_main_page(self,):
        strands_webserver.client_utils.display_relative_page(self.display_no, 'index.html')
    
    def generate_help_content(self,label, html, interaction_service):
        buttons = [(label, interaction_service)]
        content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, self.service_prefix)        
        strands_webserver.client_utils.display_content(self.display_no, content)
        

