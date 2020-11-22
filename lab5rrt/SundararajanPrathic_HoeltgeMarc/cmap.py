#Marc Hoeltege 
#Prathic Sundararajan
import json
import threading

from utils import *
import rrt_vector
import random

class CozMap:
    """Class representing a map for search algorithms.

        Features include: start location, goal location, obstacles, and path storage
        Configuration is loaded from json file supplied at object creation
        Designed to be thread-safe

        Attributes:
        width -- width of map, in mm
        height -- height of map, in mm
    """

    def __init__(self, fname, node_generator):
        with open(fname) as configfile:
            # Load dimensions from json file
            config = json.loads(configfile.read())
            self.width = config['width']
            self.height = config['height']

            # Initially empty private data, please access through functions below
            self._start = Node(tuple(config['start']))
            self._goals = [Node(tuple(coord)) for coord in config['goals']]
            self._obstacles = []
            self._nodes = []  # node in RRT
            self._node_paths = []  # edge in RRT
            self._solved = False
            self._node_generator = node_generator
            self._smooth_path = []
            self._smoothed = False

            # Read in obstacles
            for obstacle in config['obstacles']:
                self._obstacles.append([Node(tuple(coord)) for coord in obstacle])

            # For coordination with visualization
            self.lock = threading.Lock()
            self.updated = threading.Event()
            self.changes = []

    def is_inbound(self, node):
        """Check if node is within legitimate range

            Arguments:
            node -- grid coordinates
        """
        if ((node.x >= 0) and (node.y >= 0) and (node.x < self.width) and (node.y < self.height)):
            return True
        else:
            return False

    def is_collision_with_obstacles(self, line_segment):
        """Check if a line segment intersects with any obstacles

            Arguments:
            line_segment -- a tuple of two node
        """
        line_start, line_end = line_segment
        for obstacle in self._obstacles:
            num_sides = len(obstacle)
            for idx in range(num_sides):
                side_start, side_end = obstacle[idx], obstacle[(idx + 1) % num_sides]
                if is_intersect(line_start, line_end, side_start, side_end):
                    return True
        return False

    def is_inside_obstacles(self, node):
        """Check if a node is inside any obstacles

            Arguments:
            node -- the query node
        """
        for obstacle in self._obstacles:
            num_sides = len(obstacle)
            is_inside = True
            for idx in range(num_sides):
                side_start, side_end = obstacle[idx], obstacle[(idx + 1) % num_sides]
                if get_orientation(side_start, side_end, node) == 2:
                    is_inside = False
                    break
            if is_inside:
                return True
        return False

    def get_size(self):
        """Return the size of grid
        """
        return self.width, self.height

    def get_nodes(self):
        """Return all nodes in RRT
        """
        return self._nodes

    def get_goals(self):
        """Return list of goals
        """
        return self._goals

    def get_num_nodes(self):
        """Return number of nodes in RRT
        """
        return len(self._nodes)

    def set_start(self, node):
        """Set the start cell

            Arguments:
            node -- grid coordinates of start cell
        """
        if self.is_inside_obstacles(node) or (not self.is_inbound(node)):
            print("start is not updated since your start is not legitimate\nplease try another one\n")
            return
        self.lock.acquire()
        self._start = Node((node.x, node.y))
        self.updated.set()
        self.changes.append('start')
        self.lock.release()

    def get_start(self):
        """Get start
        """
        return self._start

    def add_goal(self, node):
        """Add one more goal

            Arguments:
            node -- grid coordinates of goal cell
        """
        if self.is_inside_obstacles(node) or (not self.is_inbound(node)):
            print("goal is not added since your goal is not legitimate\nplease try another one\n")
            return
        self.lock.acquire()
        self._goals.append(node)
        self.updated.set()
        self.changes.append('goals')
        self.lock.release()

    def add_obstacle(self, nodes):
        """Add one more obstacles

            Arguments:
            nodes -- a list of four nodes denoting four corners of a rectangle obstacle, in clockwise order
        """

        self.lock.acquire()
        self._obstacles.append(nodes)
        self.updated.set()
        self.changes.append('obstacles')
        self.lock.release()

    def get_random_valid_node(self):
        """Get one random node which is inbound and avoids obstacles
        """
        return self._node_generator(self)

    def add_node(self, node):
        """Add one node to RRT
        """
        self.lock.acquire()
        self._nodes.append(node)
        self.updated.set()
        self.changes.append('nodes')
        self.lock.release()

    def add_path(self, start_node, end_node):
        """Add one edge to RRT, if end_node is close to goal, mark problem is solved

            Arguments:
            start_node -- start node of the path
            end_node -- end node of the path
        """
        if self.is_collision_with_obstacles((start_node, end_node)):
            return
        self.lock.acquire()
        end_node.parent = start_node
        self._nodes.append(end_node)
        self._node_paths.append((start_node, end_node))

        for goal in self._goals:
            if get_dist(goal, end_node) < 15 and (not self.is_collision_with_obstacles((end_node, goal))):
                goal.parent = end_node
                self._nodes.append(goal)
                self._node_paths.append((end_node, goal))
                self._solved = True
                break

        self.updated.set()
        self.changes.extend(['node_paths', 'nodes', 'solved' if self._solved else None])
        self.lock.release()
    

    def is_solved(self):
        """Return whether a solution has been found
        """
        return self._solved

    def is_solution_valid(self):
        """Check if a valid has been found
        """
        if not self._solved:
            return False
        cur = None
        for goal in self._goals:
            cur = goal
            while cur.parent is not None:
                cur = cur.parent
            if cur == self._start:
                return True
        return False

    def get_smooth_path(self):
        if self._smoothed:
            return self._smooth_path[:]
        self.lock.acquire()
        self._smooth_path = self.compute_smooth_path()
        self._smoothed = True
        self.updated.set()
        self.changes.append('smoothed')
        self.lock.release()
        return self._smooth_path[:]

    def compute_smooth_path(self, limit=75):
        ############################################################################
        # TODO: please enter your code below.
        """
        Loop a fixed amount of times:
  -Take two random different nodes that are not right next to each other
  -If there isn't an obstacle between them:
    -modify your path so that the two random nodes are connected to each other, and the nodes in between are skipped
  -If the distance between the two nodes are bigger than limit in
  step_from_to(), just create multiple nodes between the two random nodes, as
  shown in the image below.
    """
        #print("CHECK", self.is_solution_valid())
        num_times = 200
        #print('Num of path', len(self.get_path()))
        for loop_counter in range(num_times):
            curr_path = self.get_path()
            #print("Time number", loop_counter)
            num_nodes = len(curr_path) - 1
            rand_node_1_index = random.randint(0, num_nodes)
            rand_node_2_index = random.randint(0, num_nodes)
            if rand_node_2_index < rand_node_1_index:
                temp = rand_node_1_index
                rand_node_1_index = rand_node_2_index
                rand_node_2_index = temp
            if rand_node_2_index == rand_node_1_index:
                #rand_node_2_index = random.randint(0, num_nodes)
                continue
            
                
 #           rand_node_1, rand_node_2 = random.choice(self._node_paths)
             #random.choice(self._node_paths)
            rand_node_1 = curr_path[rand_node_1_index]
            rand_node_2 = curr_path[rand_node_2_index]
            #print("First NodeX", rand_node_1.x,"First NodeY", rand_node_1.y)
            #print("Second NodeX", rand_node_2.x,"Secomd NodeY", rand_node_2.y)
            if not self.is_collision_with_obstacles((rand_node_1, rand_node_2)):
                #self._node_paths[rand_node_2][0].parent
                new_curr_node = rand_node_1
                #print("found a new switch")
                '''
                1 --> 2 --> 3

                '''
                while (get_dist(new_curr_node, rand_node_2) > limit): #one sec brushing atm 
                    new_curr_node2 = rrt_vector.step_from_to(new_curr_node,rand_node_2,limit)
                    new_curr_node2.parent = new_curr_node
                    new_curr_node = new_curr_node2
                    #print("Finding node in between bc greater than 75.", get_dist(new_curr_node, rand_node_2))
                    #print("First NodeX", new_curr_node.x,"First NodeY", new_curr_node.y)
                    #print("Second NodeX", rand_node_1.x,"Secomd NodeY", rand_node_1.y)
                #print(get_dist(rand_node_2, new_curr_node))
                rand_node_2.parent = new_curr_node
            #print(self.get_path())
            #self._node_paths = self.get_path()
        #temporary code below to be replaced
        path = self.get_path()
        #print('returning')
        return path

        ############################################################################
        
        
    def get_path(self):
        
        final_path = None
        
        while final_path is None:
            path = []
            cur = None
            for goal in self._goals:
                cur = goal
                while cur.parent is not None:
                    path.append(cur)
                    cur = cur.parent
                    #print("ParentX", cur.x)
                    #print("ParentY", cur.y)
                if cur == self._start:
                    path.append(cur)
                    break
            final_path = path[::-1]
        
        return final_path

    def is_solved(self):
        """Return whether a solution has been found
        """
        return self._solved

    def is_solution_valid(self):
        """Check if a valid has been found
        """
        if not self._solved:
            return False
        cur = None
        for goal in self._goals:
            cur = goal
            while cur.parent is not None:
                cur = cur.parent
            if cur == self._start:
                return True
        return False

    def reset_paths(self):
        """Reset the grid so that RRT can run again
        """
        self.clear_solved()
        self.clear_nodes()
        self.clear_node_paths()
        self.clear_smooth_path()

    def clear_smooth_path(self):
        """Clear solved state
        """
        self.lock.acquire()
        self._smoothed = False
        self._smooth_path = []
        self.updated.set()
        self.lock.release()

    def clear_solved(self):
        """Clear solved state
        """
        self.lock.acquire()
        self._solved = False
        for goal in self._goals:
            goal.parent = None
        self.updated.set()
        self.changes.append('solved')
        self.lock.release()

    def clear_nodes(self):
        """Clear all nodes in RRT
        """
        self.lock.acquire()
        self._nodes = []
        self.updated.set()
        self.changes.append('nodes')
        self.lock.release()

    def clear_node_paths(self):
        """Clear all edges in RRT
        """
        self.lock.acquire()
        self._node_paths = []
        self.updated.set()
        self.changes.append('node_paths')
        self.lock.release()

    def clear_goals(self):
        """Clear all goals
        """
        self.lock.acquire()
        self._goals = []
        self.updated.set()
        self.changes.append('goals')
        self.lock.release()

    def clear_obstacles(self):
        """Clear all obstacle
        """
        self.lock.acquire()
        self._obstacles = []
        self.updated.set()
        self.changes.append('obstacles')
        self.lock.release()
