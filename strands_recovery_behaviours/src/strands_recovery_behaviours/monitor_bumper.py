import rospy

from monitored_navigation.monitor_state import MonitorState


from scitos_msgs.msg import MotorStatus



class MonitorBumper(MonitorState):
    def __init__(self):
        MonitorState.__init__(self, "/motor_status", MotorStatus,  self.cb)
    
    """ Test the message and decide exit or not """
    def cb(self,  ud,  msg):
        # using msg.bumper_pressed does not work properly because sometimes the
        # bumper is pressed but no change of state is published
        if msg.motor_stopped and not msg.free_run:
            #self.get_logger().log_bump()
            return False
        else:
            return True

