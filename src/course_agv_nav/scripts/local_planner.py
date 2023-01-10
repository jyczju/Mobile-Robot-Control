#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf
from global_planner import GlobalPlanner

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import LaserScan

import dwa

from threading import Lock,Thread
import time
import math

ROBOT_TF_NAME = "/robot_base" # "/robot_tf"

def limitVal(minV,maxV,v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v

class LocalPlanner:
    def __init__(self):
        self.arrive = 0.4
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.laser_lock = Lock()

        self.plan_config = dwa.Config()
        self.plan_config.robot_type = dwa.RobotType.rectangle
        self.dwa = dwa.DWA(self.plan_config)
        c = self.plan_config
        self.threshold = 1.5*c.max_speed*c.predict_time

        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.planner_thread = None
        self.ob = None

        self.need_exit = False
        # self.gp = GlobalPlanner()
        pass
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", ROBOT_TF_NAME, rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map',ROBOT_TF_NAME,rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw
        ind = self.goal_index
        # self.goal_index = len(self.path.poses)-1
        # print("len = ",len(self.path.poses))
        while ind < len(self.path.poses)-1:
            p = self.path.poses[ind].pose.position
            dis = math.hypot(p.x-self.x,p.y-self.y)
            # print('mdgb;; ',len(self.path.poses),ind,dis)
            # print("ind = ",ind)
            if dis < self.threshold:
                self.goal_index = ind
                if dis < 0.7:
                    self.goal_index = ind + 1
                # print("goal_index",self.goal_index)
                # print("dis",dis)
                # print("threhold",self.threshold)
                break
            ind += 1
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        lgoal = self.tf.transformPose(ROBOT_TF_NAME, goal)
        self.plan_goal = np.array([lgoal.pose.position.x,lgoal.pose.position.y])
        self.goal_dis = math.hypot(self.x-self.path.poses[-1].pose.position.x,self.y-self.path.poses[-1].pose.position.y)

    def laserCallback(self,msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100,100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment*i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a)*r,math.sin(a)*r])
        self.laser_lock.release()
        pass

    def updateObstacle1(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        
        self.plan_ob = self.gp.detec_ob()
        dis = self.plan_ob.copy()
        dis[:,0] -= self.x
        dis[:,1] -= self.y
        dis = dis**2
        dis = np.sum(dis,1)
        index = np.where(dis < 2.0)
        self.plan_ob = self.plan_ob[index]
        
        trans = np.array([[math.cos(self.yaw),-math.sin(self.yaw),self.x],[math.sin(self.yaw),math.cos(self.yaw),self.y],[0,0,1]])
        trans = np.linalg.inv(trans)
        temp = np.ones([len(self.plan_ob),1])
        self.plan_ob = np.concatenate((self.plan_ob,temp),axis = 1)
        
        self.plan_ob = np.dot(trans,self.plan_ob.T)
        self.plan_ob = self.plan_ob.T
        self.plan_ob = self.plan_ob[:,0:2]
        
        self.plan_ob = np.array(self.ob) if self.ob is not None else None
        self.laser_lock.release()
        pass
        
    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob) if self.ob is not None else None
        self.laser_lock.release()
        pass
    def pathCallback(self,msg):
        self.need_exit = True
        time.sleep(0.1)
        self.path = msg
        self.planner_thread = Thread(target=self.planThreadFunc)
        self.initPlanning()
        self.planner_thread.start()

    def initPlanning(self):
        self.goal_index = 0
        # self.vx = 0.0
        # self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.goal = np.array([cx[0],cy[0]])
        self.plan_cx,self.plan_cy = np.array(cx),np.array(cy)
        self.plan_goal = np.array([cx[-1],cy[-1]])
        self.plan_x = np.array([0.0,0.0,0.0,self.vx,self.vw])
        pass
    def planThreadFunc(self):
        print("running planning thread!!")
        rr = rospy.Rate(10)
        self.need_exit = False
        while not self.need_exit:
            self.planOnce()
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                self.publishVel(True)
                self.planner_thread = None
                self.dwa.plot_u()
                break
            rr.sleep()
        print("exit planning thread!!")
        self.publishVel(True)
        self.planner_thread = None
        pass
    def planOnce(self):
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [0.0,0.0,0.0,self.vx,self.vw]
        # self.plan_x = [self.x,self.y,self.yaw,self.vx,self.vw]
        # Update obstacle
        self.updateObstacle()
        #print("plan_ob")
        #print(self.plan_ob)
        u, _ = self.dwa.plan(self.plan_x,self.plan_goal,self.plan_ob)
        alpha = 0.9
        self.vx = u[0]*alpha+self.vx*(1-alpha)
        self.vw = u[1]*alpha+self.vw*(1-alpha)
        # print("mdbg; ",u)
        self.publishVel()
        pass

    def publishVel(self,zero = False):
        if zero:
            self.vx = 0
            self.vw = 0
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
