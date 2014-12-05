import rospy

from monitored_navigation.monitor_state import MonitorState


from std_msgs.msg import Bool
from sensor_msgs.msg import Joy



class MonitorPause(MonitorState):
    def __init__(self, is_paused=False):
        
        #if is_paused monitors for resume, else monitors for pause
        self.is_paused=is_paused
        
        self.pub=rospy.Publisher("monitored_navigation/pause_requested", Bool, queue_size=1)
        
        #pause with joy
        self.pad_paused=False
        rospy.Subscriber("/teleop_joystick/joy",Joy,self.joy_cb)
    
       
        MonitorState.__init__(self, "/monitored_navigation/pause_requested", Bool,  self.monitor_cb)
        
        
    
    def monitor_cb(self,  ud,  msg):
        if self.is_paused:
            return msg.data
        else:
            return not msg.data


    def joy_cb(self,msg):
        if msg.buttons[4]==0:
            self.pad_paused=False
        else:
            self.pad_paused=True
        self.pub.publish(self.pad_paused)

