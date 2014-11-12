"""Generates a database of grasps
"""
#libs and modules
from openravepy import *
from itertools import groupby
import numpy, time, analyzegrasp3d, scipy
import tf, csv, os
import operator, random

#ROS
import rospy, roslib
from moveit_msgs.msg import * 
from geometry_msgs.msg import * 
from trajectory_msgs.msg import *


class ORGraspGeneration:
	def __init__(self, viewer=False):
		self.env = None
		self.target = None
		self.grasp_list = None
	
	def setup_environment(self, object_name, viewer=False):
		if self.env == None:
			self.env = Environment()
			self.env.Load(roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/env/target_scene.env.xml')
		
		if viewer:
			if self.env.GetViewer() == None:
				self.env.SetViewer('qtcoin')
			else:
				print "Viewer already loaded"
		else:
			print "Not using a viewer for OpenRAVE"
		
		if self.target == None:
			self.target = self.env.ReadKinBodyURI(roslib.packages.get_pkg_dir('cob_pick_place_action')+'/files/meshes/'+str(object_name)+'.stl')
			self.env.Add(self.target,True)
		
		if self.target.GetName() != object_name:
			print "Changing the target object"
			self.env.Remove(self.target)
			self.target = self.env.ReadKinBodyURI(roslib.packages.get_pkg_dir('cob_pick_place_action')+'/files/meshes/'+str(object_name)+'.stl')
			self.env.Add(self.target,True)
		
		print "Environment set up!"
	
	
	def generate_grasps(self, object_name, replan=False):
		#robot
		robot = self.env.GetRobots()[0]
		manip = robot.GetManipulator('arm')
		gmodel = databases.grasping.GraspingModel(robot,self.target)

		#TCP - transformed to hand wrist
		tool_trafo = manip.GetLocalToolTransform()
		tool_trafo[2,3] = 0.0
		manip.SetLocalToolTransform(tool_trafo)


		### Options for GraspGeneration
		#~ friction = None
		#~ preshapes = None
		#~ manipulatordirections = None
		#~ approachrays = None
		#~ standoffs = None
		#~ rolls = None
		#~ avoidlinks = None
		#~ graspingnoise = None
		#~ plannername = None
		#~ normalanglerange = 0
		#~ directiondelta=0
		#~ translationstepmult=None
		#~ finestep=None
		####
		
		
		options = self.configure_grasp_generation()
		
		
		
		
		now_plan = time.time()
		
		### Do actual GraspGeneration
		if gmodel.load():
			if replan == True:
				print "Replanning database"
				gmodel.autogenerate(options)
			print "Successfully loaded a database"
		else:
			print "No database available"
			print "Generating a database"
			gmodel.autogenerate(options)
		
		
		#time diff
		end_plan = time.time()
		time_difference = int(end_plan - now_plan)
		print "GraspGeneration took: ", time_difference
		
		
		
		
		### GetValidGrasps now
		#Return all validgrasps
		#@OPTIONAL: We can return a desired number of grasps
		#computeValidGrasps(startindex=0, checkcollision=True, checkik=True, checkgrasper=True, backupdist=0.0, returnnum=inf)
		#Note: setting backupdist does somehow not effect the validgrasps list???
		
		validgrasps, validindicees = gmodel.computeValidGrasps(checkcollision=True, checkik=False)
		#validgrasps, validindicees = gmodel.computeValidGrasps(backupdist=0.025, checkcollision=True, checkik=False)	#opt1
		#validgrasps, validindicees = gmodel.computeValidGrasps(backupdist=0.0, checkcollision=True, checkik=False)		#opt2
		print "TotalNumValidGrasps: ",len(validgrasps)
		
		#prevent from saving an empty file
		if len(validgrasps) == 0:
			print('No valid grasps generated')
			return []
			databases.grasping.RaveDestroy()
		
		#~ backupdist_array = [0.025, 0.05, 0.075, 0.1]
		#~ num_grasps=numpy.inf
		#~ #num_grasps = 20
		#~ 
		#~ validgrasps = []
		#~ validindicees = []
		#~ 
		#~ 
		#~ for bd in backupdist_array:
			#~ ###GRASP-PLANNING
			#~ #Return all validgrasps
			#~ #@OPTIONAL: We can return a desired number of grasps
			#~ #computeValidGrasps(startindex=0, checkcollision=True, checkik=True, checkgrasper=True, backupdist=0.0, returnnum=inf)
			#~ validgrasps_bd, validindicees_bd = gmodel.computeValidGrasps(checkcollision=True, checkik=False, backupdist=bd, returnnum=num_grasps)
			#~ 
			#~ print "Backupdist: ",bd
			#~ print "NumValidGrasps: ",len(validgrasps_bd)
			#~ 
			#~ validgrasps.extend(validgrasps_bd)
			#~ validindicees.extend(validindicees_bd)
		#~ 
		#~ print "TotalNumValidGrasps: ",len(validgrasps)
		
		
		
		##Write all validgrasps to file
		grasps_to_file = []
		meta_info = []
		meta_info.append(object_name)
		meta_info.append(time)
		grasps_to_file.append(meta_info)
		
		#@todo: for debug, remove
		for graspnmb in range(0,len(validgrasps)):
		
			#for segmentation fault purposes
			auto_fill_rest = False
			
			grasp_to_file = []
			grasp_to_file.append(graspnmb) #to file grasp number for id
			grasp_num = int(graspnmb)
			
			#calculate metric and final joint configuration	
			try:
				print "Grasp: ",graspnmb
				contacts,finalconfig,mindist,volume = gmodel.runGrasp(grasp=validgrasps[grasp_num], forceclosure=True)
			except:
				print "something went wrong"
				mindist = 0.0
				volume = 0.0
			
			#show grasp before calculating the correct transformation
			direction = gmodel.getGlobalApproachDir(validgrasps[graspnmb])
			#gmodel.showgrasp(validgrasps[graspnmb],collisionfree=True)
			transf = []
			
			with gmodel.GripperVisibility(manip):
				with self.env:
					gmodel.setPreshape(validgrasps[graspnmb])
					Tgrasp = gmodel.getGlobalGraspTransform(validgrasps[graspnmb],collisionfree=True)
					Tdelta = numpy.dot(Tgrasp,numpy.linalg.inv(manip.GetEndEffectorTransform()))
					DOFValues = robot.GetActiveDOFValues()
					DOFValues[manip.GetGripperIndices()] = finalconfig[0][manip.GetGripperIndices()]
					robot.SetDOFValues(DOFValues)
					robot.SetTransform(numpy.dot(Tdelta,robot.GetTransform()))
					self.env.UpdatePublishedBodies()
			with self.target:
				self.target.SetTransform(numpy.eye(4))
				with gmodel.GripperVisibility(manip):
					with self.env:
						gmodel.setPreshape(validgrasps[graspnmb])
						Tgrasp = gmodel.getGlobalGraspTransform(validgrasps[graspnmb],collisionfree=True)
						Tdelta = numpy.dot(Tgrasp,numpy.linalg.inv(manip.GetEndEffectorTransform()))
						for link in manip.GetChildLinks():
							link.SetTransform(numpy.dot(Tdelta,link.GetTransform()))
						self.env.UpdatePublishedBodies()
						# wait while environment is locked?
						transf = manip.GetEndEffectorTransform()
						
			grasp_to_file.append(finalconfig[0][manip.GetGripperIndices()])
			grasp_to_file.append(transf)
			#gmodel.showgrasp(validgrasps[graspnmb],collisionfree=True)
			grasp_to_file.append(mindist)
			grasp_to_file.append(volume)
			
			forceclosure = validgrasps[graspnmb][gmodel.graspindices['forceclosure']]
			grasp_to_file.append(forceclosure) 
			grasp_to_file.append(validindicees[graspnmb])
			grasp_to_file.append(direction)
			#print "GlobApproachDir: ",gmodel.getGlobalApproachDir(validgrasps[i])
			#SDH Joint Values - Beginnend mit Daumen(1) und die Zwei Finger GUZS nummeriert(2)(3)
			#[Fingerwinkel(2), Fingerknick(2), Fingerrotation(2)(3), Fingerwinkel(3), Fingerknick(3), Fingerwinkel(1), Fingerknick(1)]
			grasps_to_file.append(grasp_to_file)
			
			gmodel.getGlobalApproachDir(validgrasps[graspnmb])
		
		#Create a csv file if needed
		analyzegrasp3d.or_to_csv(grasps_to_file) 
		
		print('Finished.')
		databases.grasping.RaveDestroy()
		return len(validgrasps)
	
	
	
	def configure_grasp_generation(self):
		### Options for GraspGeneration
		#~ 'preshapes'
		#~ 'manipulatordirections'
		#~ 'boxdelta'
		#~ 'spheredelta'
		#~ 'standoffs'
		#~ 'rolls'
		#~ 'friction'
		#~ 'avoidlinks'
		#~ 'graspingnoise'
		#~ 'plannername'
		#~ 'normalanglerange'
		#~ 'directiondelta'
		#~ 'translationstepmult'
		#~ 'finestep'
		#~ 'numthreads'
		####
		
		#preshape --- default is 'something strange' ?
		preshape_array = []
		#preshape = '1.047 -0.785 1.047 -0.785 1.047 -0.785 1.047' # spheric_open
		#preshape_array.append(preshape)
		preshape = '0.0 -0.9854 0.9472 -0.9854 0.9472 -0.9854 0.9472' # cylindric_open
		preshape_array.append(preshape)
		preshape = '0.0 -0.9854 0.47 -0.9854 0.47 -0.9854 0.47' # fingers_half_straight_open
		preshape_array.append(preshape)
		#preshape = '0.0 -0.9854 0.0 -0.9854 0.0 -0.9854 0.0' # fingers_straight_open
		#preshape_array.append(preshape)
		options.preshapes = numpy.array(preshape_array)
		
		#manipulatordirections --- default is [0.0,1.0,0.0] ?
		#~ md_array = []
		#~ direction1 = '0.0 0.0 1.0' #along z-axis -- points 'out of sdh'
		#~ md_array.append(direction1)
		#~ options.manipulatordirections = numpy.array(md_array)
		
		#boxdelta --- default is 0.02
		options.boxdelta = 0.05
		
		#spheredelta --- default is 0.1
		#~ only used if boxdelta is not set
		
		#standoffs --- default is [0.0, 0.025] --- 0.0 is bad as we will hit the object!
		standoff_array = [0.025, 0.075]
		options.standoffs = numpy.array(standoff_array)
		
		#rolls --- default is [0.0, pi/2, pi, 3*pi/2, 2*pi]
		
		#fricion
		options.friction = 1.0 #coefficient of static friction in graspit: rubber-<xobject> = 1
		
		#avoidlinks --- default is None ?
		
		#graspingnoise --- ?
		
		#plannername --- ?
		
		#normalanglerange --- default is 0.0 - used for computeXXXApproachRays
		options.normalanglerange = 0.0
		#options.normalanglerange = 15
		
		#directiondelta --- default is 0.4 - used for computeXXXApproachRays
		options.directiondelta = 1
		#options.directiondelta = 5
		
		#translationstepmult --- default is None ?
		
		#finestep
		
		#numthreads --- default is 1
		#options.numthreads = 4		#--- multiple threads somehow not working
		
		return options
	
	
	#check if a database with the object_id exists
	def check_database(self, object_name):
		#Begins here to read the grasp .csv-Files
		path_in = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/database/'+object_name+'/'+object_name+'.csv'

		#Check if path exists
		if os.path.exists(path_in):
			return True
		else: 
			return False
	
	
	
	def get_grasp_list(self, object_name, sort_by_quality=False):
		#Begins here to read the grasp .csv-Files
		path_in = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/database/'+object_name+'/'+object_name+'.csv'

		#Check if path exists
		try:
			with open(path_in) as f: pass
		except IOError as e:
			rospy.logerr("The path or file does not exist: "+path_in)

		#If exists open with dictreader
		reader = csv.DictReader( open(path_in, "rb"), delimiter=',')
		
		if sort_by_quality:
			#sorting for threshold
			self.grasp_list = sorted(reader, key=lambda d: float(d['eps_l1']), reverse=True)
		else:
			self.grasp_list = sorted(reader, key=lambda d: float(d['id']), reverse=False)
	
	
	#get the grasps
	def get_grasps(self, object_name, grasp_id=0, num_grasps=0, threshold=0):
		#open database
		self.get_grasp_list(object_name)
		
		#check for grasp_id and return 
		if grasp_id > 0:
			if grasp_id < len(self.grasp_list):
				#print self._fill_grasp_msg(self.grasp_list[grasp_id])
				return [self._fill_grasp_msg(self.grasp_list[grasp_id])]
			else:
				print "Grasp not available"
				return []
	
		#sorting for threshold
		sorted_list = sorted(self.grasp_list, key=lambda d: float(d['eps_l1']), reverse=True)

		#calculate max_grasps
		if num_grasps > 0:
			max_grasps=min(len(sorted_list),num_grasps)
		else:
			max_grasps=len(sorted_list)
			
		#grasp output
		selected_grasp_list = []
		for i in range(0,max_grasps):
			if threshold > 0 and float(sorted_list[i]['eps_l1']) >= threshold:
				selected_grasp_list.append(self._fill_grasp_msg(sorted_list[i]))
			elif threshold == 0:
				selected_grasp_list.append(self._fill_grasp_msg(sorted_list[i]))
			else:
				pass

		#print len(selected_grasp_list)
		return selected_grasp_list
	
	
	#fill the grasp message of ROS
	def _fill_grasp_msg(self, grasp):
		
		#grasp posture
		joint_config = JointTrajectory()
		joint_config.header.stamp = rospy.Time.now()
		#joint_config.header.frame_id = ""
		joint_config.joint_names = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
		print "Optimize grasp_configuration"
		for joint_name in joint_config.joint_names:
			point = JointTrajectoryPoint()
			point.positions.append(float(grasp[joint_name]))
			point.velocities.append(0.0)
			point.accelerations.append(0.0)
			point.time_from_start = rospy.Duration(3.0)
			joint_config.points.append(point)
		## WARNING: in hydro the message format has changed, thus the following does not work anymore
		##joint_config.position = [float(grasp['sdh_knuckle_joint']), float(grasp['sdh_finger_12_joint']), float(grasp['sdh_finger_13_joint']), float(grasp['sdh_finger_22_joint']), float(grasp['sdh_finger_23_joint']), float(grasp['sdh_thumb_2_joint']), float(grasp['sdh_thumb_3_joint'])]
		##print joint_config.position
		#joint_config.position = [float(grasp['sdh_knuckle_joint']), 0.92*float(grasp['sdh_finger_12_joint']), float(grasp['sdh_finger_13_joint']), 0.92*float(grasp['sdh_finger_22_joint']), float(grasp['sdh_finger_23_joint']), 0.92*float(grasp['sdh_thumb_2_joint']), float(grasp['sdh_thumb_3_joint'])]
		#print joint_config

		#pregrasp posture
		pre_joint_config = JointTrajectory()
		pre_joint_config.header.stamp = rospy.Time.now()
		pre_joint_config.joint_names = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
		cyl_open = [0.0, -0.9854, 0.9472, -0.9854, 0.9472, -0.9854, 0.9472]
		#pre_joint_config.header.frame_id = ""
		for i in range(len(pre_joint_config.joint_names)):
			point = JointTrajectoryPoint()
			point.positions.append(cyl_open[i])
			point.velocities.append(0.0)
			point.accelerations.append(0.0)
			point.time_from_start = rospy.Duration(3.0)
			pre_joint_config.points.append(point)
		print pre_joint_config

		#grasp pose
		grasp_pose = PoseStamped()
		grasp_pose.header.stamp = rospy.Time.now()
		#grasp_pose.header.frame_id = ""
		grasp_pose.pose.position.x = float(grasp['pos-x'])*0.001 #mm to m
		grasp_pose.pose.position.y = float(grasp['pos-y'])*0.001 #mm to m
		grasp_pose.pose.position.z = float(grasp['pos-z'])*0.001 #mm to m
		grasp_pose.pose.orientation.x = float(grasp['qx'])
		grasp_pose.pose.orientation.y = float(grasp['qy'])
		grasp_pose.pose.orientation.z = float(grasp['qz'])
		grasp_pose.pose.orientation.w =	float(grasp['qw'])
   
		#grasp
		grasp_out = Grasp()
		grasp_out.id = grasp['id']
		grasp_out.pre_grasp_posture = pre_joint_config
		grasp_out.grasp_posture = joint_config
		grasp_out.grasp_pose = grasp_pose
		grasp_out.grasp_quality = float(grasp['eps_l1'])
		grasp_out.max_contact_force = 0

		return grasp_out
	
	
	def show_grasp(self, object_name, grasp_id, sort_by_quality=False):
		self.setup_environment(object_name, viewer=True)
		
		self.get_grasp_list(object_name, sort_by_quality)
		
		
		#robot
		robot = self.env.GetRobots()[0]
		manip = robot.GetManipulator('arm')
		gmodel = databases.grasping.GraspingModel(robot,self.target)

		#TCP - transformed to hand wrist
		tool_trafo = manip.GetLocalToolTransform()
		tool_trafo[2,3] = 0.0
		manip.SetLocalToolTransform(tool_trafo)

		#Show the grasps
		#SetDOFValues
		grasp = self.grasp_list[int(grasp_id)]
		print 'Grasp ID: '+str(grasp['id'])
		print 'Grasp Quality epsilon: '+str(grasp['eps_l1'])
		print 'Grasp Quality volume: '+str(grasp['vol_l1'])
		#print grasp

		with gmodel.GripperVisibility(manip):
			dof_array = [0]*27
			dof_array[7:13] =[float(grasp['sdh_finger_22_joint']),float(grasp['sdh_finger_23_joint']),float(grasp['sdh_knuckle_joint']),float(grasp['sdh_finger_12_joint']),float(grasp['sdh_finger_13_joint']),float(grasp['sdh_thumb_2_joint']),float(grasp['sdh_thumb_3_joint'])]
			robot.SetDOFValues(numpy.array(dof_array))

			#matrix from pose
			mm_in_m = 0.001
			pos = [float(grasp['pos-x'])*mm_in_m, float(grasp['pos-y'])*mm_in_m, float(grasp['pos-z'])*mm_in_m]  
			quat = [float(grasp['qw']), float(grasp['qx']), float(grasp['qy']), float(grasp['qz'])]
			Tgrasp = matrixFromPose(quat+pos)

			#transform robot
			Tdelta = numpy.dot(Tgrasp,numpy.linalg.inv(manip.GetEndEffectorTransform()))
			for link in manip.GetChildLinks():
				link.SetTransform(numpy.dot(Tdelta,link.GetTransform()))

			#publish update of scene
			time.sleep(1)
			self.env.UpdatePublishedBodies()


