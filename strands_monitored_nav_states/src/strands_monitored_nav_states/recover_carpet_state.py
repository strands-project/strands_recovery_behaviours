import rospy
import smach
from monitored_navigation.recover_state import RecoverState
from geometry_msgs.msg import Twist

   
class CarpetState(RecoverState):
    def __init__(self,
                 name="recover_stuck_on_carpet",
                 is_active=True,
                 max_recovery_attempts=float("inf")):
        RecoverState.__init__(self,
                        name=name,
                        outcomes=['recovered_with_help', 'recovered_without_help','not_recovered_with_help', 'not_recovered_without_help', 'preempted'],
                        is_active=is_active,
                        input_keys=['n_fails'],
                        output_keys=['n_fails'],
                        max_recovery_attempts=max_recovery_attempts
                        )
        self.was_helped=False
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_cmd = Twist()
        
        self.last_call=rospy.Time.now()
        

    def active_execute(self,userdata):
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'
        
        current_time=rospy.Time.now()
        if current_time-self.last_call < rospy.Duration(60):
            userdata.n_fails+=1
        self.last_call=current_time
        
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
                
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'

        #TODO: find way to check if behaviour was successful       
        if True:     
            return 'recovered_without_help'
        else:
            return 'not_recovered_with_help'
