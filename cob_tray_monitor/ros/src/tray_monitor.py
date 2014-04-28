#!/usr/bin/python
import roslib; roslib.load_manifest('cob_tray_monitor')
import rospy
import math
from std_msgs.msg import *
from sensor_msgs.msg import *
from cob_srvs.srv import *
from collections import deque

class Monitor():
    
    def __init__(self):        
        self.pub = rospy.Publisher("occupied", Bool)
        rospy.Subscriber("range_0", Range, self.range1_callback)
        rospy.Subscriber("range_1", Range, self.range2_callback)
        rospy.Subscriber("range_2", Range, self.range3_callback)
        rospy.Subscriber("range_3", Range, self.range4_callback)
        rospy.Service('occupied', Trigger, self.srv_callback)
        self.range1 = Range()
        self.queue1 = deque([1]) # init with arbitrary high value
        self.avg1 = sum(self.queue1)/len(self.queue1)
        self.range2 = Range()
        self.queue2 = deque([1])
        self.avg2 = sum(self.queue2)/len(self.queue2)
        self.range3 = Range()
        self.queue3 = deque([1])
        self.avg3 = sum(self.queue3)/len(self.queue3)
        self.range4 = Range()
        self.queue4 = deque([1])
        self.avg4 = sum(self.queue4)/len(self.queue4)
        self.occupied = False
        self.queueLen = 5 # lower: higher sensitivity, but also higher noise
        
        if not rospy.has_param('distance_limit'):
            rospy.logerr("no distance_limit specified")
            exit(1)

        self.limit = rospy.get_param('distance_limit')
        print self.limit
    
    def srv_callback(self,req):
        res = TriggerResponse()
        # res.success.data = self.occupied
        averages = [self.avg1, self.avg2, self.avg3, self.avg4]
        res.success.data = any(map(lambda x: x <= self.limit, averages))
        return res

    def range1_callback(self,msg):
        self.range1 = msg
        self.queue1.append(msg.range)
        if len(self.queue1)>=self.queueLen:
            self.queue1.popleft()
        self.avg1 = sum(self.queue1)/len(self.queue1)
    
    def range2_callback(self,msg):
        self.range2 = msg
        self.queue2.append(msg.range)
        if len(self.queue2)>=self.queueLen:
            self.queue2.popleft()
        self.avg2 = sum(self.queue2)/len(self.queue2)
    
    def range3_callback(self,msg):
        self.range3 = msg
        self.queue3.append(msg.range)
        if len(self.queue3)>=self.queueLen:
            self.queue3.popleft()
        self.avg3 = sum(self.queue3)/len(self.queue3)
    
    def range4_callback(self,msg):
        self.range4 = msg
        self.queue4.append(msg.range)
        if len(self.queue4)>=self.queueLen:
            self.queue4.popleft()
        self.avg4 = sum(self.queue4)/len(self.queue4)

    def publish_state(self):
        msg = Bool()
        # msg.data = False;
        averages = [self.avg1, self.avg2, self.avg3, self.avg4]
        # print ', '.join(map(str,averages))
        msg.data = any(map(lambda x: x <= self.limit, averages))
        # if self.range1.range <= self.limit:
        #     msg.data = True;
        # if self.range2.range <= self.limit:
        #     msg.data = True;
        # if self.range3.range <= self.limit:
        #     msg.data = True;
        # if self.range4.range <= self.limit:
        #     msg.data = True;
        #print msg.data
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
