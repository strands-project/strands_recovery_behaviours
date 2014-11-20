import rospy

from monitored_navigation.monitor_state import MonitorState

from scitos_msgs.msg import BarrierStatus



class MonitorMagneticStrip(MonitorState):
    def __init__(self):
        MonitorState.__init__(self, "/barrier_status", BarrierStatus,  self.cb)

    
    """ Test the message and decide exit or not """
    def cb(self,  ud,  msg):
        return not msg.barrier_stopped

