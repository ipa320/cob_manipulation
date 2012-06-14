#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_tray_monitor')
import rospy
import math
from std_msgs.msg import *
from sensor_msgs.msg import *
from cob_srvs.srv import *

class Monitor():
    
    def __init__(self):        
        self.pub = rospy.Publisher("occupied", Bool)
        rospy.Subscriber("/tray_sensors/range_1", Range, self.range1_callback)
        rospy.Subscriber("/tray_sensors/range_2", Range, self.range2_callback)
        rospy.Subscriber("/tray_sensors/range_3", Range, self.range3_callback)
        rospy.Subscriber("/tray_sensors/range_4", Range, self.range4_callback)
        rospy.Service('occupied', Trigger, self.srv_callback)
        self.range1 = Range()
        self.range2 = Range()
        self.range3 = Range()
        self.range4 = Range()
        self.occupied = True
        
        if not rospy.has_param('distance_limit'):
            rospy.logerr("no distance_limit specified")
            exit(1)

        self.limit = rospy.get_param('distance_limit')
    
    def srv_callback(self,req):
        res = TriggerResponse()
        res.success.data = self.occupied
        return res

    def range1_callback(self,msg):
        self.range1 = msg
    
    def range2_callback(self,msg):
        self.range2 = msg
    
    def range3_callback(self,msg):
        self.range3 = msg
    
    def range4_callback(self,msg):
        self.range4 = msg

    def publish_state(self):
        msg = Bool()
        if self.range1.range <= self.limit:
            msg.data = True;
        if self.range2.range <= self.limit:
            msg.data = True;
        if self.range3.range <= self.limit:
            msg.data = True;
        if self.range4.range <= self.limit:
            msg.data = True;

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('tactile_sensors')
    rospy.sleep(0.5)
    
    m = Monitor()
    rospy.loginfo("tray monitor running")
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.publish_state()
        r.sleep()
