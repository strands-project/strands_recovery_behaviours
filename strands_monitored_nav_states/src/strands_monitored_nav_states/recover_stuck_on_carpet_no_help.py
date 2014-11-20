import rospy

import smach
from monitored_navigation.recover_state_machine import RecoverStateMachine

from geometry_msgs.msg import Twist

from mongo_logger import MonitoredNavEventClass



class RecoverStuckOnCarpetNoHelp(RecoverStateMachine):
    def __init__(self):
        RecoverStateMachine.__init__(self)
        self.state=CarpetState()
        
        with self:
            smach.StateMachine.add('CARPET_STATE',
                                   self.state,
                                   transitions={'preempted':'preempted',
                                                'recovered_without_help':'recovered_without_help',
                                                'preempted':'preempted'})
   
class CarpetState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['recovered_with_help', 'recovered_without_help','not_recovered_with_help', 'not_recovered_without_help', 'preempted'])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self.vel_cmd = Twist()
        

    def execute(self,userdata):
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'
        
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="carpet_recovery")
            
        #small forward vel to unstuck robot
        self.vel_cmd.linear.x=0.8
        self.vel_cmd.angular.z=0.4
        for i in range(0,4): 
            self.vel_pub.publish(self.vel_cmd)
            self.vel_cmd.linear.x=self.vel_cmd.linear.x-0.2
            self.vel_cmd.angular.z=self.vel_cmd.angular.z-0.2  
            rospy.sleep(0.2)
            
        self.vel_cmd.linear.x=0.0
        self.vel_cmd.angular.z=0.0
        self.vel_pub.publish(self.vel_cmd)
        
        self.nav_stat.finalize(False,0)
        self.nav_stat.insert()
        
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'

        #TODO: find way to check if behaviour was successful       
        if True:     
            return 'recovered_without_help'
        else:
            return 'not_recovered_with_help'
