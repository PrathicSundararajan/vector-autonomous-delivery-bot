import anki_vector
import os
from anki_vector.events import Events
from anki_vector import annotate, events
from anki_vector.util import degrees, distance_mm, distance_inches, speed_mmps
import math
import sys
import time

from lab5rrt.cmap import *
from lab5rrt.gui import *
from lab5rrt.utils import *

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    ############################################################################
    
    #############################################################################
    # Instructors Solution
    if get_dist(node0, node1) < limit:
        return node1
    else:
        theta = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        return Node((node0.x + limit * np.cos(theta), node0.y + limit * np.sin(theta)))


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    ############################################################################

    #############################################################################
    # Instructors Solution
    if np.random.rand() < 0.05:
    #if np.random.rand() < 0.00:
        
        return Node((cmap.get_goals()[0].x, cmap.get_goals()[0].y))

    else:
        while True:
            rand_node = Node((np.random.uniform(cmap.width),\
                     np.random.uniform(cmap.height)))
            if cmap.is_inbound(rand_node) \
                    and (not cmap.is_inside_obstacles(rand_node)):
                break
        return rand_node
    ############################################################################


def RRT(cmap, start):
    # cmap.add_node(start)
    # map_width, map_height = cmap.get_size()
    # while (cmap.get_num_nodes() < MAX_NODES):
    #     ########################################################################
    #     # TODO: please enter your code below.
    #     # 1. Use CozMap.get_random_valid_node() to get a random node. This
    #     #    function will internally call the node_generator above
    #     # 2. Get the nearest node to the random node from RRT
    #     # 3. Limit the distance RRT can move
    #     # 4. Add one path from nearest node to random node
    #     #
    #     rand_node = None
    #     nearest_node = None
    #     pass
    #     ########################################################################
    #     time.sleep(0.01)
    #     cmap.add_path(nearest_node, rand_node)
    #     if cmap.is_solved():
    #         break

    # path = cmap.get_path()
    # smoothed_path = cmap.get_smooth_path()

    # if cmap.is_solution_valid():
    #     print("A valid solution has been found :-) ")
    #     print("Nodes created: ", cmap.get_num_nodes())
    #     print("Path length: ", len(path))
    #     print("Smoothed path length: ", len(smoothed_path))
    # else:
    #     print("Please try again :-(")
    
    ############################################################################
    # instructors solution
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        rand_node = node_generator(cmap)
        #rand_node = cmap.get_random_valid_node()
        nearest_node_dist = np.sqrt(map_height ** 2 + map_width ** 2)
        nearest_node = None
        for node in cmap.get_nodes():
            if get_dist(node, rand_node) < nearest_node_dist:
                nearest_node_dist = get_dist(node, rand_node)
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


def VectorPlanning():
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    serial = os.environ.get('ANKI_ROBOT_SERIAL', None)
    with anki_vector.Robot(serial=serial) as robot:
    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    #Add final position as goal point to cmap, with final position being defined as a point that is at x = 3/4 of the map width and y = 3/4 of the map height
    #you can get map width and map weight from cmap.get_size()
        vector_pos = cmap.get_start() #get start position. This will be [50, 35] if we use emptygrid for actual robot
        vector_angle = 0
        map_width, map_height = cmap.get_size()
        final_goal_center = Node((map_width*1/2, map_height*1/2))
        cmap.add_goal(final_goal_center) #adding a goal first to be the center
        print("center added as goal")
        
        #reset the current stored paths in cmap
        #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
        #get path from the cmap
        cmap.reset_paths()
        RRT(cmap, cmap.get_start()) #get a path based on cmap target and start position
        path = cmap.get_smooth_path() #smooth path
        print("path created")
        
        
        #marked and update_cmap are both outputted from detect_cube_and_update_cmap(robot, marked, vector_pos).
        #and marked is an input to the function, indicating which cubes are already marked
        #So initialize "marked" to be an empty dictionary and "update_cmap" = False
        marked = {}
        update_cmap = False
        
        #while the current cosmo position is not at the goal:
        while vector_pos not in cmap.get_goals():
        
            #break if path is none or empty, indicating no path was found
            if (path is None or len(path)==0):
                print("path is none") #sanmesh
                break
            
            # Get the next node from the path
            #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
            #you can calculate the angle to turn through a trigonometric function
            next_pos = path.pop(0)
            print('x: {0:.2f}, y: {1:.2f}'.format(next_pos.x, next_pos.y))
            angle = np.arctan2(next_pos.y - vector_pos.y, next_pos.x - vector_pos.x)
            print("driving robot to next node in path")
            if abs(angle - vector_angle) > 0.01:
                robot.behavior.turn_in_place(anki_vector.util.Angle(radians=angle - vector_angle))
            robot.behavior.drive_straight(distance_mm(get_dist(vector_pos, next_pos)),speed_mmps(30))
            # Update the current vector position (vector_pos and vector_angle) to be new node position and angle
            vector_pos = next_pos
            vector_angle = angle
        
            # Set new start position for replanning with RRT
            cmap.set_start(vector_pos)

            #detect any visible obstacle cubes and update cmap
            print("detect_cube_and_update_cmap")
            update_cmap, goal_center, marked = detect_cube_and_update_cmap(robot, marked, vector_pos)
            
            #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths
            if update_cmap:
                # Found the goal
                cmap.reset_paths()
                RRT(cmap, cmap.get_start())
                path = cmap.get_smooth_path()

def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    local_vec = np.array([[node.x], [node.y], [1]])
    global_T_local = np.array([[np.cos(local_angle), -np.sin(local_angle), local_origin.x],
                               [np.sin(local_angle), np.cos(local_angle), local_origin.y],
                               [0, 0, 1]])
    global_vec = global_T_local.dot(local_vec)
    return Node((int(global_vec[0]), int(global_vec[1])))


def detect_cube_and_update_cmap(robot, marked, vector_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        vector_pose -- provides the robot's pose in G_Arena
                 vector_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    vector_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((vector_pos.x+dx, vector_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class RobotThread(threading.Thread):
    """Thread to run vector code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        VectorPlanning()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset_paths()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        #creates cmap based on empty grid json
        #"start": [50, 35],
        #"goals": [] This is empty
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
