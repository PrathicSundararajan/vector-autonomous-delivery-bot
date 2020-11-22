#Marc Hoeltege 
#Prathic Sundararajan
import anki_vector
from anki_vector.events import Events
from anki_vector import annotate, events
from anki_vector.util import degrees, distance_mm, distance_inches, speed_mmps
import math
import sys
import time
import random
from cmap import *
from gui import *
from utils import *
import os
from anki_vector.behavior import MIN_HEAD_ANGLE, MAX_HEAD_ANGLE


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
    
    if get_dist(node0, node1) < limit:
        node1.parent = node0
        return node1
    #https://stackoverflow.com/questions/24596949/get-the-point-which-in-the-line-form-atan2-and-distance
    qx = node0.x - node1.x
    qy = node0.y - node1.y
    qq = limit / math.sqrt(qx**2 + qy ** 2)
    node2 = Node((node0.x * (1 - qq) + node1.x*qq, node0.y * (1 - qq) +node1.y*qq), parent=node0)
    #print(get_dist(node0, node2))
    #print("This one run")
    #print("Node 0", node0.x, node0.y)
    #print("Node 1", node1.x, node1.y)
    #print("Node 2", node2.x, node2.y)
    return node2
    
    
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    # pass
    ############################################################################
    random_percentage = random.random() * 100
    #print(random_percentage)
    if random_percentage >= 95:
        #print("Found Goal State")
        #print("Generated goal")
        return Node(cmap.get_goals()[0].coord)

    while not rand_node or not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        rand_node = Node((random.randint(0, cmap.get_size()[0]), random.randint(0, cmap.get_size()[1])))

    if not cmap.is_inbound(rand_node):
        print("OUTSIDE random")
    return rand_node

def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        min_dist = sys.maxsize
        nearest_node = None
        for node in cmap.get_nodes():
            if get_dist(rand_node, node) < min_dist and get_dist(rand_node, node) != 0:
                nearest_node = node
                min_dist = get_dist(rand_node, node)        
        rand_node = step_from_to(nearest_node, rand_node)
        #if not cmap.is_inbound(rand_node):
        #print("New Node", len(cmap.get_nodes()))
        #    continue
        #cmap.add_node(rand_node)

        ########################################################################
        time.sleep(0.01)
        #print("New Node Parent Is:", nearest_node.x,nearest_node.y)
        #print("New Node Parent Expect:", rand_node.parent.x,rand_node.parent.y)
        #print(get_dist(nearest_node, rand_node))
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            print('solved')
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
    serial = os.environ.get('ANKI_ROBOT_SERIAL', None)
    with anki_vector.Robot(serial=serial) as robot:
    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    #Add final position as goal point to cmap, with final position being defined as a point that is at the center of the arena 
    #you can get map width and map weight from cmap.get_size
        cmap_width, cmap_height = cmap.get_size()
        cmap.add_goal(Node((cmap_width/2, cmap_height/2)))
        #reset the current stored paths in cmap
        cmap.reset_paths()
        #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
        #get path from the cmap
        RRT(cmap, cmap.get_start())
        path = cmap.get_smooth_path()
    
    
        #marked and update_cmap are both outputted from detect_cube_and_update_cmap(robot, marked, cozmo_pos).
        #and marked is an input to the function, indicating which cubes are already marked
        #So initialize "marked" to be an empty dictionary and "update_cmap" = False
        marked = {}
        update_cmap = False
        next_node = 1

        robot.behavior.set_head_angle(MIN_HEAD_ANGLE)
        #while the current cosmo position is not at the goal:
        prev_angle = 0
        while True: #TODO: might error
            #(robot.pose.position.x, robot.pose.position.y) != cmap.get_goals()[0]
            #break if path is none or empty, indicating no path was found
            if path is None or path == []:
                print('no path found')
                break
            
            
            # Get the next node from the path
            #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
            #you can calculate the angle to turn through a trigonometric function
            if (len(path) > next_node):
                move_to = path[next_node]
                robot_curr_x  = path[next_node - 1].x
                robot_curr_y  = path[next_node - 1].y
                #robot_curr_z  = robot.pose.position.z
                #end_angle = math.atan((move_to.y - robot_curr_y)/(move_to.x - robot_curr_x))
                end_angle = math.atan2(move_to.y - robot_curr_y, move_to.x - robot_curr_x)   #x  ç≈TODO: UNSURE 
                dist_to_go = get_dist(move_to, Node(((robot_curr_x, robot_curr_y))))
                print('Turning to next node')
                end_angle *= 57.29                 
                print(end_angle - prev_angle)
                robot.behavior.turn_in_place(degrees(end_angle - prev_angle))
                prev_angle = end_angle
                time.sleep(2)
                print('Driving to next node')
                robot.behavior.drive_straight(distance_mm(dist_to_go),speed_mmps(100))
                time.sleep(1)
                # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle 
                vector_pos = path[next_node]

                # Set new start position for replanning with RRT
                cmap.set_start(vector_pos)
            
            #detect any visible obstacle cubes and update cmap
            update_cmap, goal_center, marked = detect_cube_and_update_cmap(robot, marked,  Node(((path[next_node].x, path[next_node].y))))
            #print('Driving to next node')
            #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths 
            if update_cmap:
                print('cmap updated')
                #cmap.add_goal(Node(goal_center))
                cmap.reset_paths()
                
                RRT(cmap, cmap.get_start())
                path = cmap.get_smooth_path()
                next_node = 0
            
            next_node += 1
        print('reached goal')
      
    

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
    nodex = math.cos(local_angle) * node.x + -1 * math.sin(local_angle) * node.y + local_origin.x
    nodey = math.sin(local_angle) * node.x + math.cos(local_angle) * node.y + local_origin.y
    new_node = Node((nodex, nodey))
    return new_node
    new_node = None



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
    cube_padding = 45.
    vector_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:
        print('object detected')

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
    print(sys.argv)
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        print(sys.argv[i])
        if ('robot' in sys.argv[i]):
            robotFlag = True
    #print("Robot Flag", robotFlag)
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
