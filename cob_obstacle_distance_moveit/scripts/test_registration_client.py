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


import rospy
from cob_srvs.srv import SetString, SetStringRequest, SetStringResponse


if __name__ == "__main__":
    rospy.wait_for_service('/register_links')
    try:
        client = rospy.ServiceProxy('/register_links', SetString)
        req = SetStringRequest()
        req.data = "arm_left_7_link"
        res = client(req)
        print(res)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
