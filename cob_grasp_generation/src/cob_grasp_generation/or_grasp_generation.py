"""Generates a database of grasps
"""
#other libs
from openravepy import *
import numpy, time, analyzegrasp3d, scipy 

def generate_grasps(object_name,replan=True):

	#env setup
	#@TODO: replace hardcoded part
	env=Environment()
	env.Load('./../common/files/env/target_scene.env.xml')

	#Viewer - Toggle GUI 
	#env.SetViewer('qtcoin')

	#robot
	robot = env.GetRobots()[0]

	#choose object for planning
	target = env.GetKinBody(object_name)

	manip = robot.GetManipulator('arm')
	gmodel = databases.grasping.GraspingModel(robot,target)

	#TCP - transformed to hand wrist
	tool_trafo = manip.GetLocalToolTransform()
	tool_trafo[2,3] = 0.0
	manip.SetLocalToolTransform(tool_trafo)

	#camera settings
	#Tcamera=matrixFromAxisAngle([-0.931749, 0.318690, -0.174012], 126.119383)
	#Tcamera[:3,3]=[-0.256918, -0.375076, 1.171106]
	#env.GetViewer().SetCamera(Tcamera)

	#fricion
	options.friction = 0.6 #coefficient of static friction in graspit: rubber-<xobject> = 1

	#preshape
	preshape1 = '0.755 -1.57079633 0.0 -1.57079633 0.0 -1.57079633 0.0' # Cycle Open
	preshape2 = '0.0 -1.57079633 0.0 -1.57079633 0.0 -1.57079633 0.0' # Standard Preshape
	options.preshapes = numpy.array([preshape1,preshape2])

	now_plan = time.time()

	#if not gmodel.load():
	if replan == True:
		gmodel.autogenerate(options)

	#time diff
	end_plan = time.time()
	time_difference = int(end_plan - now_plan)

	###GRASP-PLANNING
	#Return all validgrasps
	#@OPTIONAL: We can return a desired number of grasps
	validgrasps, validindicees = gmodel.computeValidGrasps(checkik=False,checkcollision=True)

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
			with env:
				gmodel.setPreshape(validgrasps[graspnmb])
				Tgrasp = gmodel.getGlobalGraspTransform(validgrasps[graspnmb],collisionfree=True)
				Tdelta = numpy.dot(Tgrasp,numpy.linalg.inv(manip.GetEndEffectorTransform()))
				DOFValues = robot.GetActiveDOFValues()
				DOFValues[manip.GetGripperIndices()] = finalconfig[0][manip.GetGripperIndices()]
				robot.SetDOFValues(DOFValues)
				robot.SetTransform(numpy.dot(Tdelta,robot.GetTransform()))
				env.UpdatePublishedBodies()
		with target:
			target.SetTransform(numpy.eye(4))
			with gmodel.GripperVisibility(manip):
				with env:
					gmodel.setPreshape(validgrasps[graspnmb])
					Tgrasp = gmodel.getGlobalGraspTransform(validgrasps[graspnmb],collisionfree=True)
					Tdelta = numpy.dot(Tgrasp,numpy.linalg.inv(manip.GetEndEffectorTransform()))
					for link in manip.GetChildLinks():
						link.SetTransform(numpy.dot(Tdelta,link.GetTransform()))
					env.UpdatePublishedBodies()
					# wait while environment is locked?
					transf = manip.GetEndEffectorTransform()

		grasp_to_file.append(finalconfig[0][manip.GetGripperIndices()])
		grasp_to_file.append(transf)
		#print transf
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
	analyzegrasp3d.or_to_csv(grasps_to_file, time_difference) 

	print('Finished.')
	return grasps_to_file
	databases.grasping.RaveDestroy()
	
