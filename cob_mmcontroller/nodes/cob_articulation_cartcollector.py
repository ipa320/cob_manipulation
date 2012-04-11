#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy

import rosbag

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion 
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
from cob_mmcontroller.srv import *

import copy
import os

class cob_articulation_cartcollector:
    def __init__(self):
        try:
            #wait for services advertised by articulation_structure/src/structure_learner_srv.cpp
            rospy.wait_for_service('fit_models', 5)
            rospy.wait_for_service('get_spanning_tree', 5)
            rospy.wait_for_service('get_fast_graph', 5)
            rospy.wait_for_service('visualize_graph', 5)
            print 'Services OK'

        except rospy.ROSException:
            print 'Services not found'
            rospy.signal_shutdown('Quit')


        self.pose_pub = rospy.Publisher('track', TrackMsg)
        self.object_pub = rospy.Publisher('object', ArticulatedObjectMsg)
        rospy.Subscriber('/arm_controller/cart_state', PoseStamped, self.callback, queue_size=1)
        
        # services
        self.fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
        self.get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
        self.get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
        self.visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)

        rospy.Service('/record_track', RecordTrack, self.recordCB)
        self.record = False
        self.finished = False

        # parameter
        self.sigma_position = rospy.get_param('~sigma_position',0.01)
        self.sigma_orientation = rospy.get_param('~sigma_orientation',0.3)
        self.reduce_dofs = rospy.get_param('~reduce_dofs',1)
        self.samples = rospy.get_param('~samples',50) #keep this number of samples
        self.downsample = rospy.get_param('~downsample',False)#downsample or latest obs?

        self.object_parts = []
        self.object_msg = ArticulatedObjectMsg()

        set_param(self.object_msg, "sigma_position", self.sigma_position, ParamMsg.PRIOR)
        set_param(self.object_msg, "sigma_orientation", self.sigma_orientation, ParamMsg.PRIOR)
        set_param(self.object_msg, "reduce_dofs", self.reduce_dofs, ParamMsg.PRIOR)


    def recordCB(self, request):
        response = RecordTrackResponse()
        response.success.data = True
        if not self.record:
            print 'Starting to record trajectory'
            self.record = True
            response.error_message.data = "started recording"
        else:
            print 'Already recording trajectory and will stop now'
            self.record = False
            response.error_message.data = "stopped recording"
            while not self.finished:
                rospy.sleep(0.1)
            articulationObject_bag = ArticulatedObjectMsg()
            articulationObject_bag = self.object_msg
            file_name = os.path.join('bag_files', str(request.file_name.data) + '_articulationObject_' + '_'.join([model.name[0:2] for model in articulationObject_bag.models]) + '.bag')
            bag = rosbag.Bag(file_name, 'w')
            try:
                bag.write('object', articulationObject_bag)
                print 'Wrote articulationObject to bag file'
                response.error_message.data = response.error_message.data + " and stored bag file(s) with an object"
            finally:
                bag.close()

            for model in articulationObject_bag.models:
                model_bag = ModelMsg()
                model_bag = model
                file_name = os.path.join('bag_files', '_'.join([request.file_name.data, model.name, str(model.id)]) + '.bag')
                bag = rosbag.Bag(file_name, 'w')
                try:
                    bag.write('model', model_bag)
                    print 'Wrote model %d to bag file'%model.id
                    response.error_message.data = response.error_message.data + " and a %s model"%model.name
                finally:
                    bag.close()



        return response
            


    def callback(self, pose):
        if self.record:
            self.finished  = False
            print 'adding pose ..'
            if(len(self.object_parts)==0):
                self.object_parts = [
                    TrackMsg(
                               id=i,

                    )
                    for i in range(2)
                    ]

            identity = Pose(Point(0,0,0),Quaternion(0,0,0,1))
            self.object_parts[0].pose.append(identity)
            self.object_parts[1].pose.append(pose.pose)

            # publish current track
            rospy.loginfo('sending tracks with '+('/'.join(["%d"%len(track.pose) for track in self.object_parts]))+' poses')
            for track in self.object_parts:
                track.header = pose.header
                self.pose_pub.publish(track)
            
            
            self.object_msg.header = pose.header
            self.object_msg.parts = copy.deepcopy(self.object_parts)
            if self.downsample:
                for part in self.object_msg.parts:
                    if len(part.pose)>self.samples:
                        part.pose = [p for (i,p) in enumerate(part.pose) if i % (len(part.pose) / self.samples + 1) == 0 or i==len(part.pose)-1]
            else:
                for part in self.object_msg.parts:
                    if len(part.pose)>self.samples:
                        part.pose = part.pose[len(part.pose) - self.samples:]

            # fit model
            request = ArticulatedObjectSrvRequest()
            request.object = self.object_msg

            parts = len(self.object_parts)
            print "calling fit service"
            response = self.fit_models(request)
            print " fit service done"
            print '\n'.join(
                ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f sigma_pos=%f sigma_orient=%f'%(
                    model.name[0:2],
                    model.id/parts,
                    model.id%parts,
                    get_param(model,'bic'),
                    get_param(model,'avg_error_position'),
                    get_param(model,'avg_error_orientation'),
                    get_param(model,'sigma_position'),
                    get_param(model,'sigma_orientation')
                ) for model in response.object.models])
            
            # get fast graph
            request.object = copy.deepcopy(response.object)

            response = self.get_fast_graph(request)
            self.object_pub.publish(response.object)
            self.object_msg.models = response.object.models

            # visualize graph
            request.object = copy.deepcopy(response.object)
            response = self.visualize_graph(request)

            rospy.loginfo('received articulation model: '+(' '.join(
                ['(%s:%d,%d)'%(model.name[0:2],model.id/parts,model.id%parts) for model in response.object.models])))

            print '\n'.join(
                ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f'%(
                    model.name[0:2],
                    model.id/parts,
                    model.id%parts,
                    get_param(model,'bic'),
                    get_param(model,'avg_error_position'),
                    get_param(model,'avg_error_orientation')
                ) for model in response.object.models])
            print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f"%(
                    get_param(response.object,'dof'),
                    get_param(response.object,'dof.nominal'),
                    get_param(response.object,'bic'),
                    get_param(response.object,'avg_error_position'),
                    get_param(response.object,'avg_error_orientation')
                    )
            print "done evaluating new pose.."
            self.finished = True


def main():
    try:
        rospy.init_node('cob_articulation_cartcollector')
        cob_articulation_cartcollector();
        rospy.spin()
    except rospy.ROSInterruptException: pass 


if __name__ == '__main__':
    main()
