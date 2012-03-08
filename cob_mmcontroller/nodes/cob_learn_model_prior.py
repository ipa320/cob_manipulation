#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

import StringIO
import sys
import os

from cob_mmcontroller.cob_articulation_models_prior import *

from articulation_msgs.msg import *
from articulation_msgs.srv import *
from cob_mmcontroller.msg import *
from cob_mmcontroller.srv import *
from cob_srvs.srv import *

class cob_learn_model_prior:
    def __init__(self):

        # articulation models prior object
        self.models_prior_object = cob_articulation_models_prior()

        # action servers
        self.learnModelPrior_as = actionlib.SimpleActionServer('learn_model_prior', LearnModelPriorAction, self.learnModelPriorActionCB, False)
        self.learnModelPrior_as.start() 

    def learnModelPriorActionCB(self, goal):
        # set up and initialize action feedback and result
        result_ = LearnModelPriorResult()
        feedback_ = LearnModelPriorFeedback()
        result_.success = False
        feedback_.message = "started"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # ask whether prior models shoud be load from database
        if goal.database == "" or not os.path.isfile(goal.database):
            if self.models_prior_object.query("Do you want to load prior models from database?", ['y', 'n']) == 'y':
                # ask user to enter database to load prior models from
                goal.database = self.models_prior_object.query_database(True)
            
                # load prior models into model_learner_prior
                self.models_prior_object.load_prior_from_database(goal.database)
                feedback_.message = "Loaded prior models from database and set up model_learner"
            else:
                #self.models_prior_object.prior_changed = True
                feedback_.message = "No prior models were loaded"
        else:
            self.models_prior_object.load_prior_from_database(goal.database)
            feedback_.message = "Loaded prior models from database and set up model_learner"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # ask user how trajectory will be generated
        trajectory_generation = self.models_prior_object.query("How should the trajectory be generated: [m]anually or [a]utomatically?", ['m', 'a'])

        # if automatically execution was chosen, request parameters
        if trajectory_generation == 'a':
            feedback_.message = "trajectory will be generated automatically by cob_cartesian_trajectories_PID"
            moveModel_goal = self.models_prior_object.query_articulation_parameters()
        else:
            feedback_.message = "trajectory will be generated manually"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # wait for user interaction to start cartcollector and get model of kinematic mechanism
        # wait for keypress to start
        raw_input("Press any key to start recording")
        cartcoll_request = CartCollectorPriorRequest()
        cartcoll_response = self.models_prior_object.toggle_collector(cartcoll_request)
        feedback_.message = "Started to record trajectory and calculating model"
        self.learnModelPrior_as.publish_feedback(feedback_)
        
        # execute movement
        if trajectory_generation == 'a':
            feedback_.message = "Trajectory generation started"
            self.learnModelPrior_as.publish_feedback(feedback_)
            # start mm controller
            mm_request = TriggerRequest()
            mm_response = self.models_prior_object.start_mm(mm_request)
            # send goal to cob_cartesian_trajectories_PID's moveModel action
            self.models_prior_object.moveModel_ac.send_goal(moveModel_goal)
            self.models_prior_object.moveModel_ac.wait_for_result(rospy.Duration.from_sec(moveModel_goal.target_duration.data.secs + 0.5))
            # stop cartcollector
            cartcoll_response = self.models_prior_object.toggle_collector(cartcoll_request)
            # evaluate moveModel result
            if self.models_prior_object.moveModel_ac.get_result() == 0:
                feedback_.message = "Succeesful trajectory generation"
            elif self.models_prior_object.moveModel_ac.get_result()  == 2:
                feedback_.message = "Succeesful trajectory generation but stopped because a joint limit was almost reached"
            else:
                result_.error_message = "Trajectory generation didn't succeed"
                result_.success = False
                self.learnModelPrior_as.set_aborted(result_)
            self.learnModelPrior_as.publish_feedback(feedback_)

            
        # stop cartcollector
        # if manually execution was chosen wait for user interaction
        if trajectory_generation == 'm':
            # wait for keypress to stop
            raw_input("Press any key to stop recording")
            cartcoll_request = CartCollectorPriorRequest()
            cartcoll_response = self.models_prior_object.toggle_collector(cartcoll_request)

        # check if cartcollection went well
        if cartcoll_response.success:
            learned_model = cartcoll_response.model
        else:
            result_.error_message = "Collecting cartesian poses didn't succeed"
            result_.success = False
            self.learnModelPrior_as.set_aborted(result_)

        # TODO evaluate model

        # output prior models and learned model
        print 75*"-" + "\nPrior models:"
        self.models_prior_object.print_prior_models()
        print 75*"-" + "\nLearned model:"
        self.models_prior_object.print_model(learned_model)
        print 75*"-" + "\nVerbose:"
        prior_models_list = self.models_prior_object.get_prior_models().model
        prior_models_list.append(learned_model)
        self.models_prior_object.print_models_verbose(prior_models_list)

        # decide / ask whether to store model or not
        if self.models_prior_object.query("Do you want to store the just learned model in the prior models", ['y', 'n']) == 'y':
            if learned_model.id != -1:
                if self.models_prior_object.query("Do you want to update model %d "%learned_model.id, ['y', 'n']) == 'n':
                    learned_model.id = -1
            # store model in prior models
            self.models_prior_object.store_model_to_prior(learned_model)
        else:
            self.models_prior_object.prior_changed = False
            feedback_.message = "Didn't store learned model in prior models"
        self.learnModelPrior_as.publish_feedback(feedback_)



        # save new prior to database
        feedback_.message = "Prior models were not saved in database"
        if self.models_prior_object.prior_changed:
            # get database name if necessary
            if goal.database != "" and self.models_prior_object.query("Do you want to save the new prior models to database %s"%goal.database, ['y', 'n']) == 'y':
                    self.models_prior_object.save_prior_to_database(goal.database)
                    feedback_.message = "Prior models were saved in %s"%goal.database
            elif self.models_prior_object.query("Do you want to save the new prior models in a database", ['y', 'n']) == 'y':
                self.models_prior_object.save_prior_to_database(self.models_prior_object.query_database())
                feedback_.message = "Prior models were saved"
            else:
                if self.models_prior_object.query("Do you really want to discard the new prior models", ['y', 'n']) == 'y':
                    print "New prior model will be discard"
                else:
                    self.models_prior_object.save_prior_to_database(self.models_prior_object.query_database())
                    feedback_.message = "Prior models were saved"
        else:
            print "No new prior models were generated"
            if self.models_prior_object.query("Do you want to save the currently loaded prior models anyway", ['y', 'n']) == 'y':
                if goal.database == "":
                    self.models_prior_object.save_prior_to_database(self.models_prior_object.query_database())
                else:
                    self.models_prior_object.save_prior_to_database(goal.database)
                feedback_.message = "Prior models were saved"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # set action result
        result_.success = True
        self.learnModelPrior_as.set_succeeded(result_)



def main():
    try:
        rospy.init_node('cob_learn_model_prior')
        cob_learn_model_prior()
        rospy.spin()

    except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
    main()
