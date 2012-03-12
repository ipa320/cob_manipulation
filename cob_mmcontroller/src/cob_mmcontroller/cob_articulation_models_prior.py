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

class cob_articulation_models_prior(object):
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

        # action clients
        self.moveModel_ac = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
        self.moveModel_ac.wait_for_server()

        # service clients
        self.toggle_collector = rospy.ServiceProxy('collector_toggle', CartCollectorPrior) #?? change into action!?
        self.eval_model = rospy.ServiceProxy('model_select_eval', TrackModelSrv)
        self.store_model = rospy.ServiceProxy('model_store', TrackModelSrv)
        self.set_prior = rospy.ServiceProxy('model_prior_set', SetModelPriorSrv)
        self.get_prior = rospy.ServiceProxy('model_prior_get', GetModelPriorSrv)
        self.start_mm = rospy.ServiceProxy('/mm/start', Trigger)

        # variables
        self.prior_changed = False


    ######################################################
    # output methods
    def print_parameter(self, model_id, name, value):
        print ("ID: " + str(model_id)).ljust(7), ("NAME: " + name).ljust(30), ("VALUE: " + str(value)).ljust(30)


    def filter_parameters(self, models, param_name):
        for n in range(len(models[0].params)):
            if param_name[0] in models[0].params[n].name or param_name[1] in models[0].params[n].name:
                for model in models:
                    if model.name == models[-1].name:
                        self.print_parameter(model.id, model.params[n].name, model.params[n].value)
                print 75*"-"


    def print_model(self, model):
        print ("ID: " + str(model.id)).ljust(7), ("NAME: " + model.name).ljust(20), ("POSES: " + str(len(model.track.pose))).ljust(30)


    def print_prior_models(self):
        # print prior models
        for model in self.get_prior_models().model:
            self.print_model(model)


    def print_models_verbose(self, models):
        # filter out relevant models
        if models[-1].id != -1: # if learned model updades a prior model
            keep = []
            # keep prior model with same id as learned one
            for k in range(len(models)):
                if models[k].id == models[-1].id:
                    keep.append(models[k])
            models = keep

        # prints all evaluation parameters of all given models
        for n in range(len(models[0].params)):
            if models[0].params[n].type == 2:
                for model in models:
                    self.print_parameter(model.id, model.params[n].name, model.params[n].value)
                print 75*"-"

        # prints informative articulation parameter 
        if models[-1].name == "rotational": # for rotational articulations
            self.filter_parameters(models, ["rot_center", "rot_radius"])
        else: # for rigid and prismatic articulation
            self.filter_parameters(models, ["rigid_position", "rigid_orientation"])


    ######################################################
    # methods for user interaction
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


    ######################################################
    # load, save and store methods
    def get_prior_model_by_id(self, model_id):
        prior_models = self.get_prior_models()
        for prior_model in prior_models.model:
            if prior_model.id == model_id:
                return prior_model
        return None

    def get_prior_models(self):
        # get prior from model_learner_prior
        request = GetModelPriorSrvRequest()

        try:
            response = self.get_prior(request)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to get prior models: %s"%e)
        return response


    def store_model_to_prior(self, model):
        # adds a learned model to the prior models
        try:
            store_request = TrackModelSrvRequest()
            store_request = model
            store_response = self.store_model(store_request)
            self.prior_changed = True
        except rospy.ServiceException, e:
            self.prior_changed = False
            rospy.errlog("Failed to store learned model in prior models: %s"%e)
        return self.prior_changed


    def load_prior_from_database(self, database):
        # load prior from database and set up model_learner_prior node
        request = SetModelPriorSrvRequest()
        try:
            with open(database, 'r') as db_handle:
                saved_prior = db_handle.read()
                request.deserialize(saved_prior)
                response = self.set_prior(request)
                rospy.loginfo("%d prior model(s) loaded from %s", len(request.model), database)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to set prior models: %s"%e)
            raise
        except IOError, e:
            rospy.logerr("Failed to read prior database: %s"%e)
            raise


    def save_prior_to_database(self, database):
        # get prior from model_learner_prior and save into database
        try:
            response = self.get_prior_models()
            output = StringIO.StringIO()
            response.serialize(output)
            with open(database, "w") as dh_handle:
                dh_handle.write(output.getvalue())
            output.close()
        except (IOError, UnicodeError), e:
            rospy.logerr("Failed to write prior models to database: %s"%e)
            


    ######################################################
    # cartcollector methods
    def cartcollector_start(self):
        self.cartcollector_toggle()
        return

    
    def cartcollector_stop(self):
        return self.cartcollector_toggle()


    def cartcollector_toggle(self):
        try:
            cartcoll_request = CartCollectorPriorRequest()
            cartcoll_response = self.toggle_collector(cartcoll_request)
        except rospy.ServiceException, e:
            rospy.errlog("Toggle cartcollector service failed: %s"%e)
            raise
        
        return cartcoll_response



    ######################################################
    # evaluation method
    def evaluate_model(self, model):
        request = TrackModelSrvRequest()
        request.model = model
        try:
            return self.eval_model(request.model)
        except rospy.ServiceException, e:
            rospy.infolog("Model evaluation service failed, trying service call again")
            return self.eval_model(track_model)


    def compare_evaluation(self, prior_model, learned_model):
        try:
            prior_response = self.evaluate_model(prior_model)
            learned_response = self.evaluate_model(learned_model)
            if self.get_parameter_value(learned_response.model, 'bic') <= self.get_parameter_value(prior_response.model, 'bic'):
                return learned_response
            else:
                return prior_response

        except rospy.ServiceException, e:
            rospy.errlog("Model evaluation service failed: %s"%e)
            rospy.signal_shutdown("Evaluation Failed")
        except LookupError, e:
            rospy.errlog(e)
            rospy.signal_shutdown("Evaluation Failed")


    def get_parameter_value(self, model, param_name):
        for parameter in model.param:
            if parameter.name == param_name:
                return parameter.value
        raise LookupError("Parameter not %s found"%param_name)


