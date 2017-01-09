import rospy

from monitored_navigation.monitor_state import MonitorState

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool



class MonitorStuckOnCarpet(MonitorState):
    def __init__(self):
        self.goal_z=0
        self.current_z=0
        
        self.n_fails=0
        self.MAX_FAILS=100
  
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)   
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
 
        self.pub = rospy.Publisher('/monitored_navigation/stuck_on_carpet', Bool, queue_size=1)
        self.pub_msg=Bool(False)
        
        
        MonitorState.__init__(self, "/monitored_navigation/stuck_on_carpet", Bool,self.monitor_cb, input_keys=['n_fails'], output_keys=['n_fails'])
        


    def vel_callback(self,msg):
        self.goal_z=msg.angular.z
        if self.goal_z != 0 and self.current_z==0:
            self.n_fails=self.n_fails+1
        else:
            self.n_fails=0
        if self.n_fails>self.MAX_FAILS:
            self.pub_msg.data=True
        else:
            self.pub_msg.data=False
        self.pub.publish(self.pub_msg)    

    def odom_callback(self,msg):
        self.current_z=msg.twist.twist.angular.z
        

    """ Test the message and decide exit or not """
    def monitor_cb(self,  ud,  msg):
        if msg.data:
            ud.n_fails+=1
        return not msg.data





