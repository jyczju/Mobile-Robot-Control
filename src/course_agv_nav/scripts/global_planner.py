#!/usr/bin/env python
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan,PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

# from a_star import AStarPlanner as Planner
from rrt_star import RRT_STAR as Planner
# from rrt_star import Node
# from rrt import RRT as Planner

ROBOT_TF_NAME = "/robot_base" # "/robot_base"
MAP_TOPIC_NAME = "/map" # "/map"

class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_grid_size = 0.3
        self.plan_robot_radius = 0.75
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        # count to update map
        self.map_count = 0

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.map_sub = rospy.Subscriber(MAP_TOPIC_NAME,OccupancyGrid,self.mapCallback)
        self.collision_sub = rospy.Subscriber('/collision_checker_result',Bool,self.collisionCallback)

        # self.updateMap()
        # self.updateGlobalPose()

        pass
    def goalCallback(self,msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0)
        pass

    def collisionCallback(self,msg):
        self.replan(0)
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", ROBOT_TF_NAME, rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def replan(self,req):
        print('get request for replan!!!!!!!!')
        self.initPlanner()
        self.updateGlobalPose()
        self.plan_rx,self.plan_ry = self.planner.plan(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        self.publishPath()
        res = True
        return PlanResponse(res)
    def initPlanner(self):
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        ox,oy = np.nonzero(map_data > 50)
        self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        self.planner = Planner(self.plan_ox,self.plan_oy,self.plan_grid_size,self.plan_robot_radius)
        
    def detec_ob(self):
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        ox,oy = np.nonzero(map_data > 50)
        self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        plan_ob = np.array(list(zip(self.plan_ox, self.plan_oy)))
        # print("shape",plan_ob)
        return plan_ob

    def mapCallback(self,msg):
        self.map = msg
        pass

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[i]
            pose.pose.position.y = self.plan_ry[i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
