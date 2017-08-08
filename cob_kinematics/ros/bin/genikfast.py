#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
