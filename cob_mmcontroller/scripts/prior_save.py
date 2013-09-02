#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> ..
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
from articulation_models import track_utils
from articulation_models.track_utils import *
import StringIO
import pickle;


def main():
  rospy.init_node('model_prior_get')

  model_prior_get = rospy.ServiceProxy('model_prior_get', GetModelPriorSrv)

  request = GetModelPriorSrvRequest()

  try:
    response = model_prior_get(request)
    print "got %d prior models, saving to file"%len(response.model)
    output = StringIO.StringIO()
    response.serialize(output)
    fh = open("prior.db", 'w')
    fh.write(output.getvalue())
    fh.close()
  except rospy.ServiceException:
    print "failed to call service"
    pass



if __name__ == '__main__':
  main()

