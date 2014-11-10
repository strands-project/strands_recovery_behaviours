import rospy

from monitored_navigation.monitor_state import MonitorState


from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from strands_recovery_behaviours.srv import PauseResumeNav



class MonitorPause(MonitorState):
    def __init__(self, is_paused=False):
        
        #if is_paused monitors for resume, else monitors for pause
        self.is_paused=is_paused
        
        self.pub=rospy.Publisher("monitored_navigation/pause_requested", Bool)
        
        #pause with joy
        self.pad_paused=False
        rospy.Subscriber("/teleop_joystick/joy",Joy,self.joy_cb)
      
        #pause with service
        self.service_paused=False        
       
        MonitorState.__init__(self, "/monitored_navigation/pause_requested", Bool,  self.monitor_cb)
        
        
    def execute(self, userdata):
        pause_service = rospy.Service('/monitored_navigation/pause_nav', PauseResumeNav, self.pause_service_cb)
        result=MonitorState.execute(self, userdata)
        pause_service.shutdown()
        return result
    
    
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
        self.pub.publish(self.pad_paused or service_paused)

 
    def pause_service_cb(self, req):
        self.service_paused=req.pause
        self.pub.publish(self.pad_paused or self.service_paused)
        return []

