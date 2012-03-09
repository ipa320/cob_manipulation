#! /usr/bin/env pyhton

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.cob_articulation_models_prior import *

class cob_move_articulation_model_prior:
    def __init__(self):
        # cob_articulation_models_prior object
        self.models_prior_object = cob_articulation_models_prior()

        # action server
        self.moveModelPrior_as = actionlib.SimpleActionServer('move_model_prior', MoveModelPriorAction, self.moveModelPriorActionCB, False)
        self.moveModelPrior_as.start()

    def moveModelPriorActionCB(self, goal):
        # set up and initialize action feedback and result
        result_ = MoveModelPriorResult()
        feedback_ = MoveModelPriorFeedback()
        result_.success = False
        feedback_.message = "Move model prior action started"
        self.moveModelPrior_as.publish_feedback(feedback_)

        # load prior models from database and set up model_learner_prior
        self.models_prior_object.load_prior_from_database(goal.database)
        feedback_.message = "Loaded prior models from database %s and set up model_learner_prior"%str(goal.database)
        self.moveModelPrior_as.publish_feedback(feedback_)

        # get model according to model_id
        execute_model = self.model_prior_object.get_prior_model_by_id(goal.model_id)
        if execute_model == None:
            err_msg = "No model for found with ID %d"%goal.model_id
            feedback_.message = err_msg
            self.moveModelPrior_as.publish_feedback(feedback_)
            rospy.errlog(err_msg)
            rospy.signal_shutdown(err_msg)

        # add parameters to model
        execute_model.model.params.append(ParamMsg('action', float(goal.action_variable), 1))
        execute_model.target_duration.secs = float(goal.duration)

        # start cartcollector
        self.models_prior_object.cartcollector_start()
        feedback_.message = "Started to record trajectory and calculating model for model update"
        self.moveModelPrior_as.publish_feedback(feedback_)

        # execute movement
        print "Recording of the cartesain gripper coordinates for model update is now running"
        print "Please wait until the trajectory execution has finished, watch out and stay tuned!"
        feedback_.message = "Model movement has started"
        self.moveModelPrior_as.publish_feedback(feedback_)
        # start mm controller
        mm_request = TriggerRequest()
        mm_response = self.models_prior_object.start_mm(mm_request)
        # send model to cob_cartesian_trajectories_PID's moveModel action
        self.models_prior_object.moveModel_ac.send_goal(execute_model)
        self.models_prior_object.moveModel_ac.wait_for_result(rospy.Duration.from_sec(moveModel_goal.target_duration.secs + 0.5))
        # stop cartcollector
        cartcoll_response = self.models_prior_object.cartcollector_stop()

        # evaluate moveModel result
        if self.models_prior_object.moveModel_ac.get_result() == 0:
            feedback_.message = "Succeesful trajectory generation"
        elif self.models_prior_object.moveModel_ac.get_result()  == 2:
            feedback_.message = "Succeesful trajectory generation but stopped because a joint limit was almost reached"
        else:
            result_.error_message = "Trajectory generation didn't succeed"
            result_.success = False
            self.moveModelPrior_as.set_aborted(result_)
        self.moveModelPrior_as.publish_feedback(feedback_)

        # evaluation ?

        
        # decision

        # store to prior

        # save prior to database


def main():
    try:
        rospy.init_node('cob_move_articulation_model_prior')
        cob_move_articulation_model_prior()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
