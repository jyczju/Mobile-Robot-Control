from sys import path
from matplotlib.pyplot import step
from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, x, y, cost=0, parent=None, next=[]):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
        self.next = next  # list of son trees


class RRT_STAR(object):
    '''RRT_STAR'''

    def __init__(self, ox, oy, avoid_buffer, robot_radius):
        self.ox = ox
        self.oy = oy

        self.minx = -10
        self.maxx = 10
        self.miny = -10
        self.maxy = 10
        self.robot_size = robot_radius
        self.avoid_dist = 0
        self.STEP = 0.7

    def plan(self, start_x, start_y, goal_x, goal_y):
        # tree init, start_node can be used to generate sample_tree
        start_node, goal_node, path_x, path_y, path_dict, pathtree, obstree = self.Treeinit(
            start_x, start_y, goal_x, goal_y)
        # loop start
        for i in range(0, 1000):
            rnd_node = self.Sample()
            nearest_node = self.Nearest(
                pathtree, rnd_node, path_x, path_y, path_dict)
            # new_node's cost & parent will be specified in following procedure
            new_node, nearest_node = self.Steer(
                rnd_node, nearest_node, goal_node, path_x, path_y, path_dict, pathtree, obstree)

            if self.CollisionFree(new_node, nearest_node, obstree):
                # k must <=3 for the restriction in rewire(), otherwise rewire() needs to be rewrote
                NEAR_NODES = self.NearC(
                    new_node, pathtree, path_dict, path_x, path_y, 3)
                min_node, min_cost = self.ChooseParent(
                    NEAR_NODES, nearest_node, new_node, obstree)
                # Add new node to the path tree
                path_x, path_y, path_dict, pathtree = self.addNodeEdge(min_node, new_node, min_cost, path_x, path_y, path_dict, pathtree)
                # check whether reaches ends (need rewrite)
                if new_node.x == goal_x and new_node.y == goal_y:
                    goal_node = new_node
                    break
                # rewire the tree
                near_node, new_node = self.rewire(
                    NEAR_NODES, new_node, obstree, path_x, path_y)
                # update path_dict
                path_dict[(new_node.x, new_node.y)] = new_node
                path_dict[(near_node.x, near_node.y)] = near_node
                # path_x, path_y, pathtree = self.get_path_tree(start_node)
                # path tree need to be update after rewire!!!!
        if i >= 999:
            print('No path found!')
        # for loop ends
        # can rewrite path_x, path_y
        pa_x, pa_y, patree = self.get_path_tree(goal_node)
        pa_x, pa_y = self.pathsmooth(pa_x, pa_y, path_dict, obstree)
        # pa_x, pa_y = self.addinterpoint(pa_x, pa_y)
        # del pa_x[0]
        # del pa_y[0]
        print(pa_x, pa_y)
        return pa_x, pa_y  # road_map, sample_x, sample_y

    def Treeinit(self, start_x, start_y, goal_x, goal_y):
        '''Init the obstree, pathtree and other variables for further use'''
        # Obstacles
        obstree = KDTree(np.vstack((self.ox, self.oy)).T)
        # start_node and goal_node record the path through linked list
        start_node = Node(start_x, start_y)
        goal_node = Node(goal_x, goal_y)
        # Generate a pathtree(KDTree) that can be used in nearC(), path_x, path_y is used to search nodes in the node_dict
        path_x = [start_x]
        path_y = [start_y]
        pathtree = KDTree(np.vstack((path_x, path_y)).T)
        path_dict = {}
        path_dict[(start_x, start_y)] = start_node
        return start_node, goal_node, path_x, path_y, path_dict, pathtree, obstree

    def Sample(self):
        '''Get Random Node'''
        rnd_node = Node((random.random() * (self.maxx - self.minx)) + self.minx,
                        (random.random() * (self.maxy - self.miny)) + self.miny)
        return rnd_node

    def Nearest(self, pathtree, rnd_node, path_x, path_y, path_dict):
        '''Get the near node'''
        distance, index = pathtree.query(np.array([rnd_node.x, rnd_node.y]))
        nearest_node = path_dict[(path_x[index], path_y[index])]
        return nearest_node

    def Steer(self, rnd_node, nearest_node, goal_node, path_x, path_y, path_dict, pathtree, obstree):
        '''Find the capable node within robot's STEP
           If new_node is close enough to the goal node, change new_node's coordinate to the goal's
        '''
        # find the capable nearest node
        angle = math.atan2(rnd_node.y - nearest_node.y,
                           rnd_node.x - nearest_node.x)
        new_node = Node(math.cos(angle) * self.STEP + nearest_node.x,
                        math.sin(angle) * self.STEP + nearest_node.y)
        if len(path_x) > 1:
            # goal detection
            distance, index = pathtree.query(
                np.array([goal_node.x, goal_node.y]), min(len(path_x), 5))
            for (dis, ind) in zip(distance, index):
                if dis < self.STEP and self.CollisionFree(goal_node, path_dict[(path_x[ind], path_y[ind])], obstree):
                    new_node.x = goal_node.x
                    new_node.y = goal_node.y
                    nearest_x = path_x[ind]
                    nearest_y = path_y[ind]
                    nearest_node = path_dict[(nearest_x, nearest_y)]
                    break
        return new_node, nearest_node

    def CollisionFree(self, new_node, nearest_node, obstree):
        '''Collision Detection'''
        dx = nearest_node.x - new_node.x
        dy = nearest_node.y - new_node.y
        ang = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        x1 = new_node.x
        y1 = new_node.y
        step_size = self.robot_size
        steps = round(dis/step_size)
        for i in range(steps+1):
            distance, index = obstree.query(np.array([x1, y1]))
            if distance <= self.robot_size + self.avoid_dist:
                return False
            x1 += step_size*math.cos(ang)
            y1 += step_size*math.sin(ang)
        return True

    # need to check twice
    def NearC(self, new_node, pathtree, path_dict, path_x, path_y, k):
        '''Find k's near nodes'''
        NEAR_NODES = []
        distance, index = pathtree.query(
            np.array([new_node.x, new_node.y]), min([len(path_x), k]))
        # check whether there is only one return node
        if type(distance) != np.ndarray:
            distance = [distance]
            index = [index]
        for i in index:
            temp_node = path_dict[(path_x[i], path_y[i])]
            NEAR_NODES.append(temp_node)
        return NEAR_NODES

    # wrong
    def ChooseParent(self, NEAR_NODES, nearest_node, new_node, obstree):
        '''Choose the cloest node to the new (random) node'''
        min_node = nearest_node
        min_cost = nearest_node.cost + \
            math.hypot(new_node.x - nearest_node.x,
                       new_node.y - nearest_node.y)
        for near_node in NEAR_NODES:
            cost = near_node.cost + \
                math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)
            if self.CollisionFree(new_node, near_node, obstree) and cost < min_cost:
                min_node = near_node
                min_cost = cost
        return min_node, min_cost

    def addNodeEdge(self, min_node, new_node, min_cost, path_x, path_y, path_dict, pathtree):
        '''make the min_node and the new_node a edge'''
        new_node.cost = min_cost
        new_node.parent = min_node
        min_node.next.extend([new_node])
        new_node.next = []

        # update path_dict
        path_dict[(new_node.x, new_node.y)] = new_node
        path_dict[(min_node.x, min_node.y)] = min_node
        path_x.append(new_node.x)
        path_x.append(min_node.x)
        path_y.append(new_node.y)
        path_y.append(min_node.y)
        pathtree = KDTree(np.vstack((path_x, path_y)).T)
        return path_x, path_y, path_dict, pathtree
        # return min_node, new_node

    # must > 2
    def rewire(self, NEAR_NODES, new_node, obstree, path_x, path_y):
        '''Rewire the path'''
        for near_node in NEAR_NODES:
            cost = new_node.cost + \
                math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)
            if self.CollisionFree(new_node, near_node, obstree) and cost < near_node.cost:
                near_node.parent = new_node
                # new_node.next_node.append(near_node)
                near_node.cost = new_node.cost + \
                    math.hypot(new_node.x - near_node.x,
                               new_node.y - near_node.y)
                new_node.next.append(near_node)
                return near_node, new_node

        return near_node, new_node

    def get_path_tree(self, last_node):
        temp_node = last_node
        path_x = []
        path_y = []
        while temp_node.parent != None and temp_node.parent != 0:
            # path_x.append(temp_node)
            path_x.insert(0, temp_node.x)  # last to first
            path_y.insert(0, temp_node.y)
            temp_node = temp_node.parent
        # first element (start node)
        path_x.insert(0, temp_node.x)
        path_y.insert(0, temp_node.y)

        pathtree = KDTree(np.vstack((path_x, path_y)).T)
        return path_x, path_y, pathtree

    def pathsmooth(self, path_x, path_y, path_dict, obstree):
        '''Path Smooth'''
        while True:
            flag = True
            for i in range(0, len(path_x)-2):
                if self.CollisionFree(path_dict[(path_x[i], path_y[i])], path_dict[(path_x[i+2], path_y[i+2])], obstree):
                    del path_x[i+1], path_y[i+1]
                    # print('del')
                    flag = False
                    break
            if flag:
                break
        return path_x, path_y

    def addinterpoint(self, path_x, path_y):
        '''add interpoint, make path more smooth'''
        # print(path_x)
        if len(path_x) > 1:
            if math.hypot(path_x[-1]-path_x[-2], path_y[-1]-path_y[-2]) > 3000:
                xinsert = path_x[-2]/4+path_x[-1]*3/4
                yinsert = path_y[-2]/4+path_y[-1]*3/4
                path_x.insert(-1, xinsert)
                path_y.insert(-1, yinsert)
        # print(path_x)
        return path_x, path_y
    # def generate_road_map(self, goal_node):
    #     '''Get road_map'''
    #     node = goal_node
    #     road_map = []
    #     while node.parent != None:
    #         None
    #     return None
