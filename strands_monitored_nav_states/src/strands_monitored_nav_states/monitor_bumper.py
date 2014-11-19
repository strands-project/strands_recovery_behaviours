import rospy

from monitored_navigation.monitor_state import MonitorState

from std_msgs.msg import Bool
from scitos_msgs.msg import MotorStatus



class MonitorBumper(MonitorState):
    def __init__(self):
        MonitorState.__init__(self, "/motor_status", MotorStatus,  self.cb)
        
        self.barrier_found=False
        self.magnetic_listener = rospy.Subscriber("/magnetic_barrier", Bool,  self.barrier_cb)
        self.magnetic_helped = rospy.Subscriber("/magnetic_barrier_helped", Bool,  self.barrier_helped_cb)
    
    """ Test the message and decide exit or not """
    def cb(self,  ud,  msg):
        if self.barrier_found:
            return True
        #  msg.bumper_pressed does not  always change state when bumper is pressed
        if msg.motor_stopped and not msg.free_run:
            return False
        else:
            return True

    def barrier_cb(self, msg):
        if msg.data:
            self.barrier_found=True
    
    def barrier_helped_cb(self, msg):
        if msg.data:
            self.barrier_found=False