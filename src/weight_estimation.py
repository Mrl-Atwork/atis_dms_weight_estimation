#!/usr/bin/env python

# @author Erfan karimian

import rospy
from geometry_msgs.msg import PoseStamped
import math
import json
import nav_msgs.srv as nav_srvs
import numpy as np


with open('/home/irvan/catkin_ws/src/learning_tf2/nodes/points.json') as f:
    json_data = json.load(f)
locations = {}
for x in json_data['goals']:
    locations[int(x['count'])] = {'x': x['x'], 'y': x['y'], 'z': x['z'], 'z_o': x['z_o'], 'w': x['w']}

len_locations=len(locations)   
distance_array=np.ndarray(shape=(len_locations,len_locations), dtype=float)

def get_path(start_point,goal_point, locations):
    get_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan',nav_srvs.GetPlan)
    first_point = PoseStamped()
    first_point.header.frame_id = "map"
    first_point.pose.position.x = float(locations[start_point]['x'])
    first_point.pose.position.y = float(locations[start_point]['y'])
    first_point.pose.position.z = float(locations[start_point]['z'])
    first_point.pose.orientation.z = float(locations[start_point]['z_o'])
    first_point.pose.orientation.w = float(locations[start_point]['w'])
    end_point = PoseStamped()
    end_point.header.frame_id = "map"
    end_point.pose.position.x = float(locations[goal_point]['x'])
    end_point.pose.position.y = float(locations[goal_point]['y'])
    end_point.pose.position.z = float(locations[goal_point]['z'])
    end_point.pose.orientation.z = float(locations[goal_point]['z_o'])
    end_point.pose.orientation.w = float(locations[goal_point]['w'])
    
    path=get_plan(first_point,end_point,0.2)
    return get_distance(path)


def get_distance(path):
    first_time = True
    prev_x = 0.0
    prev_y = 0.0
    total_distance = 0.0
    for current_point in path.plan.poses:
        x = current_point.pose.position.x
        y = current_point.pose.position.y
        if not first_time:
            total_distance += math.hypot(prev_x - x, prev_y - y)
        else:
            first_time = False
            prev_x = x
            prev_y = y 
    return total_distance           

for x in locations:
    for y in locations:
        if x == y:
            print("match")
            print(x, y)
            distance = 0
            print("distance is :",distance)
            distance_array[x][y]=distance
            continue
        else:
            print(x, y) 
            distance=get_path(x,y,locations)
            print("distance is :",distance)
            distance_array[x][y]=distance



np.savetxt("/home/irvan/catkin_ws/src/atis_dms_weight_estimation/lenght.txt", distance_array, delimiter=", ",fmt='%s')
