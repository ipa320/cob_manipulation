#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

import StringIO
import sys
import os

from articulation_msgs.msg import *
from articulation_msgs.srv import *
from cob_mmcontroller.msg import *
from cob_mmcontroller.srv import *
from cob_srvs.srv import *

class cob_learn_model_prior:
    def __init__(self):
        try:
            # wait for services
            rospy.wait_for_service('collector_toggle', 5)
            rospy.wait_for_service('model_select_eval', 5)
            rospy.wait_for_service('model_store', 5)
            rospy.wait_for_service('model_prior_set', 5)
            rospy.wait_for_service('model_prior_get', 5)
            rospy.wait_for_service('/mm/start', 5)
            rospy.loginfo("Services OK")
        except:
            rospy.logerr("Service(s) not found")
            rospy.signal_shutdown("Missing services")

        # action servers
        self.learnModelPrior_as = actionlib.SimpleActionServer('learn_model_prior', LearnModelPriorAction, self.learnModelPriorActionCB, False)
        self.learnModelPrior_as.start() 
        
        # action clients
        self.moveModel_ac = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
        self.moveModel_ac.wait_for_server()

        # service clients
        self.toggle_collector = rospy.ServiceProxy('collector_toggle', CartCollectorPrior) #?? change into action!?
        self.select_model = rospy.ServiceProxy('model_select_eval', TrackModelSrv)
        self.store_model = rospy.ServiceProxy('model_store', TrackModelSrv)
        self.set_prior = rospy.ServiceProxy('model_prior_set', SetModelPriorSrv)
        self.get_prior = rospy.ServiceProxy('model_prior_get', GetModelPriorSrv)
        self.start_mm = rospy.ServiceProxy('/mm/start', Trigger)

    def learnModelPriorActionCB(self, goal):
        # variables
        prior_changed = False
        prior_db = ""
        # set up and initialize action feedback and result
        result_ = LearnModelPriorResult()
        feedback_ = LearnModelPriorFeedback()
        result_.success = False
        feedback_.message = "started"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # ask whether prior models shoud be load from database
        if self.query("Do you want to load prior models from database?", ['y', 'n']) == 'y':
            # ask user to enter database to load prior models from
            prior_db = self.query_database(True)
        
            # load prior models into model_learner_prior
            self.load_prior(prior_db)
            feedback_.message = "set up prior models"
        else:
            prior_changed = True
            feedback_.message = "no prior models was loaded"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # ask user how trajectory will be generated
        trajectory_generation = self.query("How should the trajectory be generated: [m]anually or [a]utomatically?", ['m', 'a'])

        # if automatically execution was chosen, request parameters
        if trajectory_generation == 'a':
            feedback_.message = "trajectory will be generated automatically by cob_cartesian_trajectories_PID"
            moveModel_goal = self.query_articulation_parameters()
        else:
            feedback_.message = "trajectory will be generated manually"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # wait for user interaction to start cartcollector and get model of kinematic mechanism
        # wait for keypress to start
        raw_input("Press any key to start recording")
        cartcoll_request = CartCollectorPriorRequest()
        cartcoll_response = self.toggle_collector(cartcoll_request)
        feedback_.message = "Started to record trajectory and calculating model"
        self.learnModelPrior_as.publish_feedback(feedback_)
        
        # execute movement
        if trajectory_generation == 'a':
            feedback_.message = "Trajectory generation started"
            self.learnModelPrior_as.publish_feedback(feedback_)
            # start mm controller
            mm_request = TriggerRequest()
            mm_response = self.start_mm(mm_request)
            # send goal to cob_cartesian_trajectories_PID's moveModel action
            self.moveModel_ac.send_goal(moveModel_goal)
            self.moveModel_ac.wait_for_result(rospy.Duration.from_sec(moveModel_goal.target_duration.data.secs + 0.5))
            # stop cartcollector
            cartcoll_response = self.toggle_collector(cartcoll_request)
            # evaluate moveModel result
            if self.moveModel_ac.get_result() == 0:
                feedback_.message = "Succeesful trajectory generation"
            elif self.moveModel_ac.get_result()  == 2:
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
            cartcoll_response = self.toggle_collector(cartcoll_request)

        # check if cartcollection went well
        if cartcoll_response.success:
            model = cartcoll_response.model
        else:
            result_.error_message = "Collecting cartesian poses didn't succeed"
            result_.success = False
            self.learnModelPrior_as.set_aborted(result_)

        # TODO evaluate model

        # TODO output prior models and learned model
        print "Prior models:"
        self.print_prior_models()
        print "Learned model:"
        self.print_model(model)

        # decide / ask whether to store model or not
        if self.query("Do you want to store the just learned model in the prior models", ['y', 'n']) == 'y':
            # store model in prior models
            try:
                store_request = TrackModelSrvRequest()
                store_request = model
                store_response = self.store_model(store_request)
                prior_changed = True
                feedback_.message = "Stored learned model in prior models"
            except rospy.ServiceException:
                prior_changed = False
                feedback_.message = "Failed to store learned model in prior models"
        else:
            prior_changed = False
            feedback_.message = "Didn't store learned model in prior models"
        self.learnModelPrior_as.publish_feedback(feedback_)



        # save new prior to database
        feedback_.message = "Prior models were not saved in database"
        if prior_changed:
            # get database name if necessary
            if prior_db != "" and self.query("Do you want to save the new prior models to database %s"%prior_db, ['y', 'n']) == 'y':
                    self.save_prior(prior_db)
                    feedback_.message = "Prior models were saved in %s"%prior_db
            elif self.query("Do you want to save the new prior models in a database", ['y', 'n']) == 'y':
                self.save_prior(self.query_database())
                feedback_.message = "Prior models were saved"
            else:
                if self.query("Do you really want to discard the new prior models", ['y', 'n']) == 'y':
                    print "New prior model will be discard"
                else:
                    self.save_prior(self.query_database())
                    feedback_.message = "Prior models were saved"
        else:
            print "No new prior models were generated"
            if self.query("Do you want to save the currently loaded prior models anyway", ['y', 'n']) == 'y':
                self.save_prior(self.query_database())
                feedback_.message = "Prior models were saved"
        self.learnModelPrior_as.publish_feedback(feedback_)

        # set action result
        result_.success = True
        self.learnModelPrior_as.set_succeeded(result_)



    def print_model(self, model):
        print "ID: " + str(model.id) + "\tNAME: " + model.name + "\tPOSES: " + str(len(model.track.pose)) #TODO + "\tAVG_ERROR: " bic likelihood


    def print_prior_models(self):
        # get prior from model_learner_prior and print them
        request = GetModelPriorSrvRequest()

        try:
            response = self.get_prior(request)
        except rospy.ServiceException:
            rospy.logerr("Failed to get prior models")

        for model in response.model:
            self.print_model(model)


    def query(self, question, choises):
        # let user choose 
        while True:
            choise = raw_input(question + ' [' + '/'.join(choises) + ']: ').lower()
            if choise not in choises:
                print "Invalid choise!"
            else:
                return choise


    def query_database(self, load=False):
        # let user enter prior database file
        valid_file = False
        while not valid_file:
            database = raw_input("Please enter database file name: ")
            if load:
                if os.path.isfile(database):
                    valid_file = True
                else:
                    print "File not found, please enter name of existing file to load prior from"
            else:
                if os.path.isfile(database):
                    print "%s is an existing file and will be overwritten"%database
                else:
                    print "File %s not found. Will create new file"%database
                valid_file = True
        return database


    def query_articulation_parameters(self):
        goal = ArticulationModelGoal()
        goal.model_id = 1
        if self.query("What kind of articulation should be generated? Rotational or prismatic?", ['r', 'p']) == 'r':
            goal.model.name = 'rotational'
            goal.model.params.append(ParamMsg('angle', self.query_parameter('angle'), 1))
            goal.model.params.append(ParamMsg('rot_center.x', self.query_parameter('rot_center.x'), 1))
            goal.model.params.append(ParamMsg('rot_center.y', self.query_parameter('rot_center.y'), 1))
            goal.model.params.append(ParamMsg('rot_center.z', self.query_parameter('rot_center.z'), 1))
            goal.target_duration.data.secs = self.query_parameter('target_duration')
        else:
            #TODO
            goal.model.name = 'prismatic'

        return goal
    

    def query_parameter(self, param_name):
        param_value = float(raw_input("Enter float value for parameter '%s': "%param_name))
        return param_value


    def load_prior(self, database):
        # load prior from database and set up model_learner_prior node
        request = SetModelPriorSrvRequest()

        try:
            with open(database, 'r') as db_handle:
                saved_prior = db_handle.read()

                request.deserialize(saved_prior)
                response = self.set_prior(request)
                rospy.loginfo("%d prior model(s) loaded from %s", len(request.model), database)
        except rospy.ServiceException:
            rospy.logerr("Failed to load prior models")
            pass


    def save_prior(self, database):
        # get prior from model_learner_prior and save into database
        request = GetModelPriorSrvRequest()

        try:
            response = self.get_prior(request)
            output = StringIO.StringIO()
            response.serialize(output)
            with open(database, "w") as dh_handle:
                dh_handle.write(output.getvalue())
            output.close()
        except rospy.ServiceException:
            rospy.logerr("Failed to save prior models")
            pass




def main():
    try:
        rospy.init_node('cob_learn_model_prior')
        cob_learn_model_prior()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
    main()
