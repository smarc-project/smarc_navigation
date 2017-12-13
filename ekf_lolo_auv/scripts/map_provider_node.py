#!/usr/bin/env python

import rospy
import yaml
from ekf_lolo_auv.srv import *
from IPython import embed
import numpy as np
from geometry_msgs.msg import Point32

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
