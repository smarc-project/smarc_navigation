#!/usr/bin/env python

# Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import yaml
from auv_ekf_localization.srv import *
from IPython import embed
import numpy as np
from geometry_msgs.msg import Point32

# MapServer provides a map for the EKF localization from a .yaml created in Gazebo

class MapServer:

    def __init__(self):
        self.map_srv_name = rospy.get_param(rospy.get_name() + '/map_provider_srv', '/map_service')
        self.map_file = rospy.get_param(rospy.get_name() + '/map_file', 'map.yaml')
        self.depth_map = rospy.get_param(rospy.get_name() + '/rocks_depth', -90)

        self.map_srv = rospy.Service(self.map_srv_name, map_ekf, self.provide_map)
        rospy.spin()


    def provide_map(self, req):
        self.map_resp = map_ekfResponse()
        with open(self.map_file, 'r') as stream:
            try:
                data_loaded = yaml.load(stream)
                models = data_loaded.items()
                cnt = 0
                for model in models[0][1]:
                    if model['position']['z'] < self.depth_map:
                        point = Point32()
                        point.x = model['position']['x']
                        point.y = model['position']['y']
                        point.z = model['position']['z']
                        self.map_resp.map.append(point)
                        cnt = cnt + 1 
                print "map parsed!"

            except yaml.YAMLError as exc:
                pass

        return self.map_resp
        


if __name__ == "__main__":
    
    rospy.init_node('smarc_map_server')
    try:
        MapServer()
    except rospy.ROSInterruptException:
    	pass
