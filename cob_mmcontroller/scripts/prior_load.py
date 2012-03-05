#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> ..
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
from articulation_models import track_utils
from articulation_models.track_utils import *
import StringIO

def main():
  rospy.init_node('model_prior_set')

  model_prior_set = rospy.ServiceProxy('model_prior_set', SetModelPriorSrv)

  request = SetModelPriorSrvRequest()

  try:
    fh = open("prior.db", 'r')
    str = fh.read()

    print "restoring models"
    stored_response = GetModelPriorSrvResponse()
    stored_response.deserialize(str)
    request.model = stored_response.model
    resposne = model_prior_set(request)
    print "%d prior models restored"%len(request.model)
  except rospy.ServiceException:
    print "failed to call service"
    pass



if __name__ == '__main__':
  main()


