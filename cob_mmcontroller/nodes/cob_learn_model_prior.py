#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy

import StringIO

from articulation_msgs.srv import *
from cob_mmcontroller.srv import *

class cob_learn_model_prior:
    def __init__(self):
        try:
            # wait for services
            rospy.wait_for_service('collector_toggle', 5)
            rospy.wait_for_service('model_select_eval', 5)
            rospy.wait_for_service('model_store', 5)
            rospy.wait_for_service('model_prior_set', 5)
            rospy.wait_for_service('model_prior_get', 5)
            rospy.loginfo("Services OK")
        except:
            rospy.logerr("Service(s) not found")
            rospy.signal_shutdown("Missing services")

        # service clients
        self.toggle_collector = rospy.ServiceProxy('collector_toggle', LearnModelPrior) #?? change name and into action!?
        self.select_model = rospy.ServiceProxy('model_select_eval', TrackModelSrv)
        self.store_model = rospy.ServiceProxy('model_store', TrackModelSrv)
        self.set_prior = rospy.ServiceProxy('model_prior_set', SetModelPriorSrv)
        self.get_prior = rospy.ServiceProxy('model_prior_get', GetModelPriorSrv)


    def load_prior(self, database):
        request = SetModelPriorSrvRequest()

        try:
            with open(database, 'r') as db_handle:
                saved_prior = dh_handle.read()

                request.deserialize(saved_prior)
                response = self.set_prior(request)
                rospy.loginfo("%d prior models restored from %s", len(request.model), database)
        except rospy.ServiceException:
            rospy.logerr("Failed to restore prior models")
            pass

    def save_prior(self, database):
        request = GetModelPriorRequest()

        try:
            response = self.get_prior(request)
            output = StringIO.StringIO()
            response.serialize(output)
            with open(database, "w") as dh_handle:
                dh_handle.write(output.getvalue())
            output.close()
        except rospy.ServiceException:
            rospy.logerr("Failed to store prior models")
            pass




def main():
    try:
        rospy.init_node('cob_learn_model_prior')
        cob_learn_model_prior()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
    main()
