import openravepy
from openravepy.ikfast import *
import sys
import os
import logging
import yaml
import subprocess
import roslib

if __name__ == '__main__':
	settings = yaml.load(open(sys.argv[1]))
	print settings
	format = logging.Formatter('%(levelname)s: %(message)s')
	handler = logging.StreamHandler(sys.stdout)
	handler.setFormatter(format)
	log.addHandler(handler)
	log.setLevel(logging.INFO)

	convpath = os.path.join(os.path.dirname(sys.argv[0]),'urdf_openrave')
	xml = subprocess.check_output(convpath+" --urdf %s %s %s %s "% (settings['urdf_path'],settings['name'],settings['base_link'],settings['tip_link']),shell=True)
	try:
	    env=openravepy.Environment()
	    kinbody=env.ReadRobotXMLData(xml)
	    env.Add(kinbody)
	    links = [str(l.GetName()) for l in kinbody.GetLinks()]
	    print links
	    joints = [str(j.GetName()) for j in kinbody.GetJoints()]
	    print joints
	    solver = IKFastSolver(kinbody,kinbody)
	    chaintree = solver.generateIkSolver(links.index(settings['base_link']),links.index(settings['tip_link']),[joints.index(f) for f in settings['free_joints']])
	    code=solver.writeIkSolver(chaintree,lang='cpp')
	finally:
	    openravepy.RaveDestroy()
	open(sys.argv[2],'w').write(code)