#!/usr/bin/env python3

import random
import math
import sys
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib as mpl
import matplotlib.pyplot as plt

def calc_distance(p_1, p_2):
    """
    Calculates distance between two points.
    ARGUMENTS
        p1      Point 1 coordinates; tuple
        p2      Point 2 coordinates; tuple
    OUTPUT
        Straight-line distance from p1->p2; scalar
    """
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def point_to_line(p_1, p_2, p_3):
    """
    Defines a line passing through two points. Determines whether the tangent to
    the line passing through a third point intersects between the first two. Formula is
    defined at http://paulbourke.net/geometry/pointlineplane/
    ARGUMENTS
        p_1     Point 1 coordinates; tuple
        p_2     Point 2 coordinates; tuple
        p_3     Point 3 coordinates; tuple
    OUTPUT
        r_u         p_1 -> p_2 distance intersection ratio; scalar
        tan_len     Distance from p_1 -> p_2 line to p_3; scalar
    """
    # distance from P1 -> P2
    dist = math.sqrt((p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2)

    # determine intersection ratio u
    # for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len




class RRT(object):
    """
    k
    """
    def __init__(self, side1, side2, obs_num, start_point, end_point, obstacle_list):
        """
        """
        self.side_len = side1
        self.side_width = side2
        
        self.num_obs = obs_num     # number of obstacles
        if obs_num != 0:
            self.obstacle_props = obstacle_list
        else:
            self.obstacle_props = []
        #self.obstacle_props = self.gen_obstacles(self.num_obs, 0)      # generate obstacle list
        
        if start_point < [0, 0] or start_point > [side1, side2]:
            print('Error! Start_point out of range')
            sys.exit()
        if end_point < [0, 0] or end_point > [side1, side2]:
            print('Error! End point out of range!')
            sys.exit()
        self.start = start_point
        self.end = end_point                # start and end points

        self.nodes_list = [['q0', self.start, 'None']]              # list of nodes in tree
                                                                    # [Node name, node coordinates, parent node]
        
        self.segments_list = []                                     # list of node-to-node path segments

        self.gen_tree()

        self.path_nodes, self.path_segments = [], []                # nodes/segments along the path
                                                                    # from start to finish

        self.find_path()
        
        #self.coordinates = self.list_coord()                   

    # def gen_obstacles(self, num, obs_coord_list):
    #     """
    #     Generates a given number of retangular obstacles given by the 
    #     obstacle coordinates
    #     ARGUMENTS
    #         num             Number of obstacles to generate; integer
    #         obs_coord_list  Coordinates for each obstacle
    #     """
    #     obstacles = []

    #     while len(obstacles) < num:
    #         overlap = []

    #         center = (self.side_len*random.random(), self.side_len*random.random())
    #         radius = 10*random.random()

    #         # iterate over obstacle list to check for collisions
    #         for _, props in enumerate(obstacles):
    #             if calc_distance(center, props[0]) >= radius + props[1]:
    #                 overlap.append(False)
    #             else:
    #                 overlap.append(True)

    #         if any(overlap):
    #             pass
    #         else:
    #             obstacles.append([center, radius])

    #     return obstacles

    def obstacle_check(self, point):
        """
        Checks whether a point is inside any of the obstacles.
        ARGUMENTS
            point           Point to check; tuple
        OUTPUT
            collision       Collision condition; boolean (true if collision exists)
        """
        test_point = Point(point)
        
        for _, props in enumerate(self.obstacle_props):
            ll = (props[0], props[1])
            lr = (props[0] + props [2], props[1])
            ur = (props[0]+ props[3] + props[2], props[1]+props[3] + props[2])
            ul = (props[0], props[1] +props[3])
            obstacle = Polygon([ll, lr, ur, ul])
            if test_point.within(obstacle):
                return True
            else:
                pass

        return False

    # def gen_start_end(self):
    #     """
    #     Generates start/end nodes in the RRT space and checks for collisions with obstacles.
    #     OUTPUT
    #         start           Start point; tuple
    #         end             End point/goal; tuple
    #     """
    #     start_ok, end_ok = False, False

    #     while not start_ok:
    #         start = (10 + 20*random.random(), 10 + 20*random.random())

    #         if self.obstacle_check(start):
    #             pass
    #         else:
    #             start_ok = True

    #     while not end_ok:
    #         end = (70 + 20*random.random(), 70 + 20*random.random())

    #         if self.obstacle_check(end):
    #             pass
    #         else:
    #             end_ok = True

    #     return start, end

    def find_closest(self, point):
        """
        Finds the closest existing tree node to a given point.
        ARGUMENTS
            point           Point to find closest node to
        OUTPUT
            ind             List index of closest node in tree
        """
        d_list = [calc_distance(point, node[1]) for node in self.nodes_list]

        return min(range(len(d_list)), key=d_list.__getitem__)

    def gen_node(self):
        """
        l
        """
        point_ok = False
        node_name = "q{}".format(len(self.nodes_list))

        while not point_ok:
            # generate random coordinates
            p_coords = (self.side_len*random.random(), self.side_len*random.random())

            # find parent node
            parent = self.nodes_list[self.find_closest(p_coords)]

            # print(parent)

            # x- and y-distances to random point from parent node
            d_x = p_coords[0] - parent[1][0] 
            d_y = p_coords[1] - parent[1][1]

            # magnitude of vector to closest node
            vec_mag = math.sqrt((d_x**2) + (d_y**2))

            # get new node coordinates by adding unit vector components to parent coordinates
            node = (parent[1][0] + 0.35*d_x/vec_mag, # change for spacing 
                    parent[1][1] + 0.35*d_y/vec_mag)

            # if newly created node
            if self.obstacle_check(node):
                pass
            elif node[0] < 0 or node[1] < 0:
                pass
            else:
                point_ok = True

        self.nodes_list.append([node_name, node, parent[0]])
        self.segments_list.append([parent[1], node])

        
    def path_check(self, point):
        """
        Checks for a clear straight-line path from a node to the end point.
        ARGUMENTS
            point       Point to check; tuple
        OUTPUT
            Collision condition; boolean (true if collision(s) present)
        """
        # empty list to hold collision conditions between path and individual obstacles
        path_collisions = []
        # check for collision with each obstacle
        for obs in self.obstacle_props:
            # too_close, between = False, False
            # obs_ul = obs[0] + obs[3]
            # obs_lr = obs[1] + obs[2]
            # obs_center = [obs[0] + obs_ul/2, obs[1] + obs_lr/2]
            
            def intercept_rect(x1, y1, x2, y2, x_min, y_min, x_max, y_max):
                 m = (y2 - y1)/(x2 - x1)
                 b = y1-m*x1
         
                 if (x_min <= (y_min - b)/m and (y_min - b)/m <= x_max ) or (y_min <= m*x_min + b and m*x_min + b <= y_max):
                     return True
                 elif (x_min <= (y_max - b)/m and (y_max - b)/m <= x_max ) or (y_min <= m * x_min + b and m * x_min + b <= y_max):
                     return True
                 elif (y_min <= m*x_min + b and m* x_min + b <= y_max) or ( x_min <= (y_min-b)/m and (y_min-b)/m <= x_max):
                     return True
                 elif (y_min <= m*x_max + b and m * x_max + b <= y_max) or (x_min <= (y_max-b)/m and (y_max-b)/m <= x_max):
                    return True
                 else:
                    return False
                
            xmin = obs[0]
            ymin = obs[1]
            ymax = obs[1] + obs[3]
            xmax = obs[0] + obs[2]
            
            point_x = point[0]
            point_y = point[1]
            
            end_x = self.end[0]
            end_y = self.end[1]
            
                       
            check = intercept_rect(point_x, point_y, end_x, end_y, xmin, ymin, xmax, ymax)
            print(check)
            # return tangent distance and intersection ratio between obstacle center and path to end
            #r_u, d_obs = point_to_line(point, self.end, obs_center)
 
            # determine if line segment and tangent through obstacle center intersect within segment bounds
            # if 0 <= r_u <= 1:
            #     between = True

            # determine if intersection distance is smaller than obstacle radius
            # if (d_obs <= obs[2] ) or (d_obs <= obs[3]):
            #     too_close = True

            # path is blocked if intersection is both:
            #   a) within segment bounds
            #   b) closer to obstacle center than obstacle radius length
            if check:
                path_collisions.append(True)
            else:
                path_collisions.append(False)

        return any(path_collisions)
   
   
    def gen_end_seg(self):
        """
        Generates final path segment and adds to list of segments.
        """

        self.segments_list.append([self.nodes_list[-1][1], self.end])

    def gen_tree(self):
        """
        k
        """
        done = False

        while not done:
            self.gen_node()

            if not self.path_check(self.nodes_list[-1][1]):
                done = True

        self.gen_end_seg()

        self.nodes_list.append(["q{}".format(len(self.nodes_list)), self.end, self.nodes_list[-1][0]])

    def find_path(self):
        """
        Works backward through the list of points to find the path from start to finish.
        """
        current = self.nodes_list[-1]               # set end as current node
        self.path_nodes.append(current[1])          # append end coordinates to list of path nodes

        for _, j in reversed(list(enumerate(self.nodes_list))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segments.insert(0, (j[1], current[1]))
                current = j
    
    # def list_coord(self):
    #     result = [[]]
    #     cur = self.path_nodes
    #     for i in cur:
    #         result.append(i[1])
    #     return result

# list of plotting colors
# [start, end, points, path]
COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']

# call and generate RRT
obstacle_list = [] # obstacle coordinates
ll = [10, 40] # lower left coordinate
obs_length = 0.8
obs_width = 0.8
diff = [-1.571, -1.708]
start = [0, 0];
end = [3, 1]
obs1 = [-1.149 - 0.15, -0.198 - 0.15]
obs2 = [0.173 + 0.15, -1.060 - 0.15]
obs3 = [0.111 + 0.15, 0.555 + 0.15]
obstacle_list.append([obs1[0] - diff[0], obs1[1] - diff[1],obs_length, obs_width])
obstacle_list.append([obs2[0] - diff[0], obs2[1] - diff[1], obs_length, obs_width])
obstacle_list.append([obs3[0]- diff[0], obs3[1] - diff[1], obs_length, obs_width])

# inputs: side1, side2, obs_num, start_point, end_point, obstacle_list
RRT = RRT(3.5, 3.5, 3, start, end, obstacle_list)

# =============================================================================
# with open(r'C:\Users\elisa\OneDrive\Desktop\Spring 2023\498\path_data.txt', 'w') as fp:
#     for item in RRT.path_nodes:
#         # write each item on a new line
#         fp.write("%s\n" % item)
#     print('Done')
# print(*RRT.path_nodes, sep="\n")
# =============================================================================
print(*RRT.path_nodes, sep="\n")

# # create plot image
FIG, AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
# axis limits
plt.xlim(0, RRT.side_width)
plt.ylim(0, RRT.side_len)

# plot obstacles
OBSTACLES = [plt.Rectangle((j[0], j[1]), j[2], j[3]) for i, j in enumerate(RRT.obstacle_props)]
OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
AX.add_collection(OBS_PATCHES)

# plot start and end points
plt.scatter(RRT.start[0], RRT.start[1], s=200, c=COLORS[0], marker='1')
plt.scatter(RRT.end[0], RRT.end[1], s=200, c=COLORS[1], marker='2')

# plot all nodes/edges one by one
for k in enumerate(RRT.nodes_list):
    plt.scatter(k[1][1][0], k[1][1][1], s=10, c=COLORS[2])
    if k[0] > 0:
        node_seg = mpl.collections.LineCollection(RRT.segments_list[k[0]-1:k[0]], colors=COLORS[2])
        AX.add_collection(node_seg)
   # plt.pause(0.25)

# plot path nodes/edges one by one
for m in enumerate(RRT.path_nodes):
    plt.scatter(m[1][0], m[1][1], s=10, c=COLORS[3])
    if m[0] > 0:
        path_seg = mpl.collections.LineCollection(RRT.path_segments[m[0]-1:m[0]], colors=COLORS[3])
        AX.add_collection(path_seg)
  #  plt.pause(0.1)

plt.show()
