import random
import math
import matplotlib.pyplot as plt


class RRT_planner:
    """
    Rapid Random Tree (RRT) planner
    """

    class Node:
        """
        Node for RRT
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None
            self.path_x = []
            self.path_y = []

    def __init__(self, start, goal, list_obstacles, rand_area,
                 max_branch_length=0.5, path_res=0.1, goal_sample_rate=5, max_iter=1000):
        """
        Parameters:
            start: Start Position [x,y]
            goal: Goal Position [x,y]
            list_obstacles: obstacle Positions [[x,y,size],...]
            rand_area: random Sampling Area [x_min, x_max, y_min, y_max]
            max_branch_length : maximal extension for one step
            path_res : resolution of obstacle checking in the path
            goal_sample_rate : percentage of samples that are artifically set to the goal
            max_iter: maximal number of iterations

        """
        self.start_node = self.Node(start[0], start[1])
        self.end_node = self.Node(goal[0], goal[1])
        self.rand_area = rand_area
        self.max_branch_length = max_branch_length
        self.path_res = path_res
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.list_obstacles = list_obstacles
        self.list_nodes = []
        self.radius = 5# radius to check for neighbor nodes

    def plan(self, show_anim=True):
        """
        Returns the path from goal to start.
        show_anim: flag for show_anim on or off
        """

        self.list_nodes = [self.start_node]
        for it in range(self.max_iter):
            ####### 
            # Objective: create a valid new_node according to the RRT algorithm and append it to "self.list_nodes"
            # You can call any of the functions defined lower, or add your own.

            # YOUR CODE HERE

            #######

            random_node = self.get_random_node()
            near_node = self.list_nodes[self.get_closest_node_id(self.list_nodes, random_node)]
            
            new_node = self.extend(near_node, random_node)
            near_node_cost = self.compute_dist_ang(new_node, random_node)[0]
            # first change in RRT
            neighboring_nodes = self.find_neighbors(random_node)
            for node in neighboring_nodes:
                if node[1] < near_node_cost:
                    new_node_tmp = self.extend(node[0], random_node)
                    if not(self.collision(new_node_tmp, self.list_obstacles)):
                        new_node = new_node_tmp

            if not(self.collision(new_node, self.list_obstacles)):
                self.list_nodes.append(new_node)
            
            
            # Second change
            # rewire
            self.rewire(neighboring_nodes, random_node) # used to check best and shortest path

            if show_anim and it % 5:
                self.draw_graph(random_node)

            if self.distance_to_goal(new_node.x, new_node.y) <= self.max_branch_length:
                print("Reached goal")
                return self.make_final_path(len(self.list_nodes) - 1)

            if show_anim and it % 5:
                self.draw_graph(random_node)

        return None  # cannot find path

    def extend(self, or_node, dest_node):
        """
        Returns a new node going from or_node in the direction of dest_node with maximal distance of max_branch_length. New node path goes from parent to new node with steps of path_res.
        """
        new_node = self.Node(or_node.x, or_node.y)
        dist, angle = self.compute_dist_ang(new_node, dest_node)
        dist_extension = self.max_branch_length

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if dist_extension > dist:
            dist_extension = dist

        n_expand = math.floor(dist_extension / self.path_res)

        for _ in range(n_expand):
            new_node.x += self.path_res * math.cos(angle)
            new_node.y += self.path_res * math.sin(angle)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        dist, _ = self.compute_dist_ang(new_node, dest_node)
        if dist <= self.path_res:
            new_node.x = dest_node.x
            new_node.y = dest_node.y
            new_node.path_x[-1] = dest_node.x
            new_node.path_y[-1] = dest_node.y

        new_node.parent = or_node

        return new_node

    def draw_graph(self, rnd=None, final_path = False):
        # Draw a graph of the path
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.list_nodes:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ob_x, ob_y, size) in self.list_obstacles:
            plt.plot(ob_x, ob_y, "ok", ms=30 * size)

        if final_path:
            plt.plot([x for (x, y) in final_path], [y for (x, y) in final_path], '-r')
        plt.plot(self.start_node.x, self.start_node.y, "xr")
        plt.plot(self.end_node.x, self.end_node.y, "xr")
        plt.axis([0, 7, 0, 5])
        plt.gca().invert_yaxis()
        plt.grid(True)
        plt.pause(0.01)

    def make_final_path(self, goal_ind):
        # Returns the path as the list of all the node positions in node_list, from end to start
        path = [[self.end_node.x, self.end_node.y]]
        node = self.list_nodes[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def distance_to_goal(self, x, y):
        dx = x - self.end_node.x
        dy = y - self.end_node.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def rewire(self, neighbor_nodes, random_node):
        for near_node, _, current_node_indx in neighbor_nodes:
            curr_cost = self.distance_to_goal(near_node.x, near_node.y)
            tent_cost = self.distance_to_goal(random_node.x, random_node.y) + self.compute_dist_ang(random_node, near_node)[0]
            new_node = self.extend(near_node, random_node)
            if tent_cost < curr_cost and not(self.collision(new_node, self.list_obstacles)):
                self.list_nodes[current_node_indx+1] = new_node
    
    def find_neighbors(self, new_node):
        # search in radius in self.list_nodes
        dist_other_nodes = [(other_node, self.compute_dist_ang(new_node, other_node)[0],indx) for indx,other_node in enumerate(self.list_nodes)]
        dist_other_nodes = sorted(dist_other_nodes, key = lambda x: x[1])
        neighboring_nodes = []
        for node in dist_other_nodes:
            if node[1]<self.radius:
                neighboring_nodes.append(node)
        return neighboring_nodes


    def get_random_node(self):
        # Returns a random node within random area, with the goal being sampled with a probability of goal_sample_rate %
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.rand_area[0], self.rand_area[1]),
                            random.uniform(self.rand_area[2], self.rand_area[3]))
        else:  # goal point sampling
            rnd = self.Node(self.end_node.x, self.end_node.y)
        return rnd

    @staticmethod
    def collision(node, obstacleList):
        # Returns True if collision between a node and at least one obstacle in obstacleList
        for (ob_x, ob_y, size) in obstacleList:
            list_dist_x = [ob_x - x for x in node.path_x]
            list_dist_y = [ob_y - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(list_dist_x, list_dist_y)]

            if min(d_list) <= (size/2) ** 2:
                return True 

        return False     

    @staticmethod
    def compute_dist_ang(or_node, dest_node):
        # Computes distance and angle between origin and destination nodes
        dx = dest_node.x - or_node.x
        dy = dest_node.y - or_node.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        angle = math.atan2(dy, dx)
        return d, angle

    @staticmethod
    def get_closest_node_id(list_nodes, random_node):
        # Returns index of node in list_nodes that is the closest to random_node
        dist_list = [(node.x - random_node.x) ** 2 + (node.y - random_node.y)
                 ** 2 for node in list_nodes]
        min_id = dist_list.index(min(dist_list))

        return min_id
    
import numpy as np 

class RTT_Path_Follower:
    """
    Follows a path given by RRT_Planner
    """
    def __init__(self, path, local_env):
        self.path = path
        self.env = local_env
        self.path_nodes = [RRT_planner.Node(*p) for p in path]
        self.which_path_ind = -1
        self.rotate = True

    def next_action(self):

        #######
        #
        # YOUR CODE HERE: change v and omega so that the Duckiebot keeps on following the path
        #
        #######
        # Current position and angle
        cur_pos_x = self.env.cur_pos[0]
        cur_pos_y = self.env.cur_pos[2]
        cur_angle = self.env.cur_angle
        
        dx = self.path[self.which_path_ind][0] - cur_pos_x
        dy = self.path[self.which_path_ind][1] - cur_pos_y
        target_angle = -1 * np.angle(dx + dy * 1j)

        cur_node = RRT_planner.Node(cur_pos_x, cur_pos_y) 
        dest_node = RRT_planner.Node(*self.path[self.which_path_ind]) 
        dist, _ = RRT_planner.compute_dist_ang(cur_node, dest_node)

        if self.rotate:
            v=0
            omega = (target_angle%(2*np.pi) - cur_angle%(2*np.pi) )
            if  math.fabs(omega) < 2e-2:
                self.rotate=False
                #finished rotating and now moving to destination
        else:
            v = dist
            omega= 0
            if dist < 2e-2:
                self.rotate=True
                self.which_path_ind -=1
                dx = self.path[self.which_path_ind][0] - cur_pos_x
                dy = self.path[self.which_path_ind][1] - cur_pos_y
                target_angle = -1 * np.angle(dx + dy * 1j)
                omega = (target_angle%(2*np.pi) - cur_angle%(2*np.pi) )
                v = 0
        
        return v, omega