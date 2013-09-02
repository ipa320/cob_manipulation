#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from cob_mmcontroller.srv import *
from articulation_msgs.msg import ModelMsg
from articulation_msgs.srv import *


class cob_articulation_cartcollector_prior:
    def __init__(self):
        try:
            # wait for services advertised by articulation_models/src/model_learner_prior.cpp
            rospy.wait_for_service('model_select', 5)
            print "Service OK"
        except:
            print "Service(s) not found!"
            rospy.signal_shutdown('Quit')
        
        # subscriber
        rospy.Subscriber('/arm_controller/cart_state', PoseStamped, self.cartCB, queue_size=1)

        # service clients
        self.model_select = rospy.ServiceProxy('model_select', TrackModelSrv)

        # services
        rospy.Service('/collector_toggle', CartCollectorPrior, self.collector_toggleCB)

        # variables
        self.model_recorded = ModelMsg()
        self.model_selected = ModelMsg()
        self.collect = False


    def collector_toggleCB(self, request):
        response = CartCollectorPriorResponse()
        if not self.collect:
            print "starting to collect cartesian poses"
            #reset models
            self.model_recorded = ModelMsg()
            self.model_selected = ModelMsg()

            self.collect = True
            response.error_message = "started collector"
        else:
            print "already collecting, stopping collector now"
            self.collect = False
            response.error_message = "stopped collector"

        response.success = True
        response.model = self.model_selected

        return response


    def cartCB(self, pose):
        if self.collect:
            # collect trajectory poses
            print "adding pose"
            self.model_recorded.header = pose.header
            self.model_recorded.track.pose.append(pose.pose)

            # select model
            request = TrackModelSrvRequest()
            request.model = self.model_recorded
            print "select model"
            self.model_selected = self.model_select(request).model
            print "%s model selected"%self.model_selected.name



def main():
    try:
        rospy.init_node('cob_articulation_cartcollector')
        cob_articulation_cartcollector_prior();
        rospy.spin()
    except rospy.ROSInterruptException: pass


if __name__ == '__main__':
    main()

