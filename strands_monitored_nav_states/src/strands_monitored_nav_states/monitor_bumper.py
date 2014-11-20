import rospy

from monitored_navigation.monitor_state import MonitorState

from scitos_msgs.msg import MotorStatus, BarrierStatus



class MonitorBumper(MonitorState):
    def __init__(self):
        MonitorState.__init__(self, "/barrier_status", BarrierStatus,  self.cb)
        
        self.motor_stopped=False
        self.free_run=False
        self.motor_listener = rospy.Subscriber("/motor_status", MotorStatus,  self.motor_cb)
    
    """ Test the message and decide exit or not """
    def cb(self,  ud,  msg):
        if msg.barrier_stopped:
            return True
        else:
        #msg.bumper_pressed does not  always change state when bumper is pressed
            if self.motor_stopped and not self.free_run:
                return False
            else:
                return True
                

    def motor_cb(self, msg):
        self.motor_stopped=msg.motor_stopped
        self.free_run=msg.free_run
        
    
