import rospy

import smach

from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.msg import MotorStatus


  
class RestartBumper(smach.State):
    def __init__(self):
            smach.State.__init__(self,
                                outcomes=['restarted', 'preempted'],
                                output_keys=['recovered']
                                )
            self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)
            self.motor_monitor = rospy.Subscriber("/motor_status", MotorStatus, self.bumper_monitor_cb)
            self.was_helped=False
 
    
    def bumper_monitor_cb(self, msg):
        self.is_recovered = not msg.bumper_pressed
        
    
    def execute(self, userdata):
        rospy.sleep(0.2)
        
        if self.preempt_requested(): 
            self.service_preempt()
            return 'preempted'
        
        if self.is_recovered:
            self.reset_motorstop()
            userdata.recovered=True
        else:
            userdata.recovered=False       
        return "restarted"
    
