#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Jiang Yancheng

import math
from enum import Enum

import numpy as np
import matplotlib.pyplot as plt


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.7  # [m/s]
        self.min_speed = -0.7 # [m/s]
        self.max_yawrate = 200.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.4#*4  # [m/ss]
        self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.8
        self.dir_cost_gain = 0.2
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 0.0
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.4  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


class DWA:
    def __init__(self,config):
        self.config = config
        self.u_plot = []
        pass

    def plan(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        ##TODO
        min_cost = 99999999
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # calculate dynamic window
        vmin = max(self.config.min_speed, x[3]-self.config.max_accel * self.config.dt)
        vmax = min(self.config.max_speed, x[3]+self.config.max_accel * self.config.dt)
        wmin = max(-self.config.max_yawrate, x[4]-self.config.max_dyawrate * self.config.dt)
        wmax = min(self.config.max_yawrate, x[4]+self.config.max_dyawrate * self.config.dt)

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(vmin, vmax, self.config.v_reso):
            for w in np.arange(wmin, wmax, self.config.yawrate_reso):
                u = np.array([v, w])
                trajectory = self.calc_trajectory(x, u, self.config)
                dist = self.calc_dist(trajectory, goal, ob)
                if v <= (2*dist*self.config.max_accel)**0.5 and w <= (2*dist*self.config.max_dyawrate)**0.5:
                    cost = self.calc_cost(trajectory, goal, dist)
                    if cost < min_cost:
                        min_cost = cost
                        best_u = u
                        best_trajectory = trajectory
        
        # u[0] is vx, u[1] is vw
        print(best_u)
        # print("min_cost",min_cost)
        self.u_plot.append(best_u)
        return best_u, best_trajectory

    def plot_u(self):
        plt.figure(0)
        l=plt.plot(self.u_plot)
        plt.legend(handles = l,labels = ["vx","vw"])
        plt.show()

    def calc_trajectory(self, x, u, config):
        """
        calc_trajectory
        """
        x = np.array(x)
        u = np.array(u)
        trajectory = np.array([x])
        for t in np.arange(1, config.predict_time, config.dt):
            x = self.motion(x, u, config.dt)
            trajectory = np.append(trajectory, np.array([x]), axis=0)
        return trajectory

    def motion(self, x, u, dt):
        """
        motion model
        """
        if u[1] == 0:
            x[0] += u[0] * np.cos(x[2]) * dt
            x[1] += u[0] * np.sin(x[2]) * dt
            x[2] += u[1] * dt
            x[3] = u[0]
            x[4] = u[1]
        else:
            x[0] += u[0] / u[1] * (np.sin(x[2] + u[1] * dt) - np.sin(x[2]))
            x[1] += u[0] / u[1] * (-np.cos(x[2] + u[1] * dt) + np.cos(x[2]))
            x[2] += u[1] * dt
            x[3] = u[0]
            x[4] = u[1]
        return x

    def calc_cost(self, trajectory, goal, dist):
        """
        calc_cost
        """
        cost = 0.0
        goal_dist_dx = goal[0] - trajectory[0, 0]
        goal_dist_dy = goal[1] - trajectory[0, 1]
        goal_dist = np.sqrt(goal_dist_dx**2 + goal_dist_dy**2)
        #print("goal_dist",goal_dist)

        # to goal cost
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        cost_to_goal = np.sqrt(dx**2 + dy**2)
        # print("cost_to_goal",cost_to_goal* self.config.to_goal_cost_gain)
        cost += cost_to_goal * self.config.to_goal_cost_gain
        
        # directions cost
        angle1 = math.atan2(goal[1]-trajectory[-1,1],goal[0]-trajectory[-1,0])
        angle2 = trajectory[-1,2]
        delta = abs(angle1-angle2)
        if delta > math.pi:
            delta = 2*math.pi - delta
        cost += delta * self.config.dir_cost_gain

        # speed cost
        if goal_dist > 0.7:
           cost += (trajectory[0,3] - self.config.max_speed)**2 * self.config.speed_cost_gain
           # cost += (trajectory[0,3] - 0.6)**2 * self.config.speed_cost_gain
        else:
            cost += (trajectory[0,3]+1)**2 * self.config.speed_cost_gain
        # print("speedcost",cost-cost_to_goal* self.config.to_goal_cost_gain)

        # collision cost
        if dist < 1.0:
            cost += 1.0/(dist-np.sqrt((self.config.robot_width/2)**2 + (self.config.robot_length/2)**2)-0.2) * self.config.obstacle_cost_gain
        # print("cost",cost)
        return cost

    def calc_dist(self, trajectory, goal, ob):
        """
        calc_dist
        """
        # collision cost
        if ob is not None:
            for i in range(len(trajectory)):
                min_dist = 99999999
                for ob_x, ob_y in ob:
                    dx = trajectory[i, 0] - ob_x
                    dy = trajectory[i, 1] - ob_y
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist < min_dist:
                        min_dist = dist
        else:
            min_dist = 99999999
        # print("min_dist",min_dist)
        return min_dist
