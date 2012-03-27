#! /usr/bin/env python

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
        print self.models_prior_object.print_prior_models()
        feedback_.message = "Loaded prior models from database %s and set up model_learner_prior"%str(goal.database)
        self.moveModelPrior_as.publish_feedback(feedback_)

        # get model according to model_id
        execute_model = ArticulationModelGoal()
        execute_model.model = self.models_prior_object.get_prior_model_by_id(goal.model_id)
        if execute_model.model == None:
            err_msg = "No model found with ID %d"%goal.model_id
            feedback_.message = err_msg
            self.moveModelPrior_as.publish_feedback(feedback_)
            rospy.errlog(err_msg)
            rospy.signal_shutdown(err_msg)
        print "Model, to be executed:"
        self.models_prior_object.print_models_verbose([execute_model.model])

        # add parameters to model
        execute_model.model.params.append(ParamMsg('action', float(goal.action_variable), 1))
        execute_model.target_duration = goal.duration
        execute_model.model.id = int(goal.model_id)
        execute_model.model_id = int(goal.model_id) 

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
        self.models_prior_object.moveModel_ac.wait_for_result(rospy.Duration.from_sec(goal.duration.secs + 0.5))
        # stop cartcollector
        learned_model = self.models_prior_object.cartcollector_stop()

        """# evaluate moveModel result
        traj_gen_result = self.models_prior_object.moveModel_ac.get_result()
        print "Traj_gen_result: "
        print traj_gen_result
        if traj_gen_result['exit_code'] == 0:
            feedback_.message = "Succeesful trajectory generation"
        elif traj_gen_result  == 2:
            feedback_.message = "Succeesful trajectory generation but stopped because a joint limit was almost reached"
        else:
            result_.error_message = "Trajectory generation didn't succeed"
            result_.success = False
            self.moveModelPrior_as.set_aborted(result_)
        self.moveModelPrior_as.publish_feedback(feedback_)"""

        # TODO evaluation 
        # check model id of collected cartesian model
        if learned_model.model.id == -1:
            print "Collected model id differs from executed model and doesn't fit any other prior model"
            self.models_prior_object.print_models_verbose([learned_model.model])
            if self.models_prior_object.query("Do you want to save the new prior model in a database", ['y', 'n']) == 'y':
                self.models_prior_object.store_model_to_prior(learned_model.model)
                feedback_.message = "New model was stored with ID %d"%learned_model.model.id
        elif learned_model.model.id != execute_model.model.id:
            print "Collected model id differs from executed model"
            self.models_prior_object.print_models_verbose([execute_model.model])
            self.models_prior_object.print_models_verbose([self.models_prior_object.get_prior_model_by_id(learned_model.model.id), learned_model.model])
            if self.models_prior_object.query("Do you want to update the prior model %d"%learned_model.model.id, ['y', 'n']) == 'y':
                self.models_prior_object.store_model_to_prior(learned_model.model)
                feedback_.message = "Model %d was updated"%learned_model.model.id
        else: # depending on evaluation, the model will be updated
            self.models_prior_object.print_models_verbose([execute_model.model, learned_model.model])
            if self.models_prior_object.query("Check above which model you want to store: ", ['1', '2']) == '2':
                self.models_prior_object.store_model_to_prior(learned_model.model)
                feedback_.message = "Model %d was updated"%execute_model.model.id
            else:
                #self.models_prior_object.store_model_to_prior(stored_model.model)
                feedback_.message = "Model %d was not updated, because new model was not better"%execute_model.model.id        
        self.moveModelPrior_as.publish_feedback(feedback_)

        # save prior to database
        self.models_prior_object.save_prior_to_database(goal.database)
        result_.error_message = "Model execution finished successfully"
        result_.success = True
        self.moveModelPrior_as.set_succeeded(result_)


def main():
    try:
        rospy.init_node('cob_move_articulation_model_prior')
        cob_move_articulation_model_prior()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
