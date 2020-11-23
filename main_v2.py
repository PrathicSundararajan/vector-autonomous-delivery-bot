'''
PF Converging
    Max: 40 Iterations
Go to pickup zone
    -Set up CMAP (new start and new goals and add obstacle) 
    -say ready for delivery    
Pick up cube
    -check to see if it is visible by cube
        -rotate robot if it isn't visible till it is
        -only then do you dock with the cube
Go to drop Off
    -Set up CMAP (new start and new goals and add obstacle)    
Repeat

Misc
    - make sure to do .result() for async functions
'''
# Python Imports 
import time 
import math 

# Anki Vector Imports
from anki_vector.behavior import MIN_HEAD_ANGLE, MAX_HEAD_ANGLE, MAX_LIFT_HEIGHT, MIN_LIFT_HEIGHT
from anki_vector.util import degrees, distance_mm, speed_mmps
import anki_vector
import threading
import asyncio
#from anki_vector.events import * # TODO: Might need to fix imports
#from anki_vector import *

# Particle Filter Imports
from lab4pf.grid import CozGrid
from lab4pf.particle_filter import *
from lab4pf.go_to_goal_vector import ParticleFilter, compute_odometry, marker_processing
from lab4pf.markers import detect, annotator_vector
from lab4pf.gui import GUIWindow


# RRT Imports
from lab5rrt.cmap import *
from lab5rrt.gui import Visualizer
from lab5rrt.rrt_vector_updated import node_generator, RRT





global grid, pf_gui_1, pf, cmap, stopevent, scale_factor, max_pf_iterations
global PICKUP_LOC, STORAGE_LOC, obstacle_nodes
global pf_gui_1_show, rrt_gui_show, skip_pf, sole_rotations
# need to have one of these showing for script to run
pf_gui_1_show = False
rrt_gui_show = True
skip_pf = False
sole_rotations = True
max_pf_iterations = 20
scale_factor = 25
obstacle_nodes = [Node((240, 450)), Node((350,450)), Node((350,115)), Node((240, 115))]
PICKUP_LOC = Node((8 * scale_factor, 7.5 * scale_factor))
STORAGE_LOC = Node((20.5 * scale_factor, 11.5 * scale_factor))
pf_map_filename = "./lab4pf/map_arena.json"
grid = CozGrid(pf_map_filename)
pf_gui_1 = GUIWindow(grid, show_camera=True) #TODO: pf_gui might error out
cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)


def run_particle_filter(robot):
    global grid, pf, pf_gui_1, skip_pf, sole_rotations
    if skip_pf:        
        return 13, 9, 0 
    if sole_rotations:
        for i in range(4):
            large_turn = robot.behavior.turn_in_place(degrees(45))
            large_turn.result()
            time.sleep(0.5)
            large_turn = robot.behavior.turn_in_place(degrees(90))
            large_turn.result()
            time.sleep(0.5)
            large_turn = robot.behavior.turn_in_place(degrees(45))
            large_turn.result()   
            time.sleep(0.5)
        return 13, 9, 0 
    # map
    print("Entering run pf method")
    pf = ParticleFilter(grid)
    print('running pf')
    robot.behavior.set_head_angle(degrees(0))
    robot.camera.init_camera_feed()
    camera_settings = np.array([
        [296.54,      0, 160],    # fx   0  cx
        [0, 296.54, 120],  # 0  fy  cy
        [0,      0,   1]  # 0   0   1
    ], dtype=np.float)
    m_confident = False
    not_at_goal = True
    image_raw = robot.camera.latest_image.raw_image
    last_pose = robot.pose
    t = 0
    m_x, m_y, m_h = (0, 0, 0)
    marker_annotate = annotator_vector.MarkerAnnotator(robot.camera.image_annotator)
    robot.camera.image_annotator.add_annotator('Marker', marker_annotate)
    if pf_gui_1_show:
        pf_gui_1.show_particles(pf.particles)
        pf_gui_1.show_mean(m_x, m_y, m_h, m_confident)
        pf_gui_1.updated.set()
    while not m_confident:
        t += 1
        #time.sleep(1.2)
        dx, dy, diff_heading_deg = compute_odometry(robot.pose, last_pose)  # odometery
        markers, camera_image = marker_processing(robot, camera_settings, show_diagnostic_image=False)
        if t == 1:
            length = 0
        else:
            prev_markers = markers
            length = len(prev_markers)        
        print("Run", t, len(markers))
        m_x, m_y, m_h, m_confident = pf.update((dx, dy, diff_heading_deg), markers)
        last_pose = robot.pose
        if pf_gui_1_show:
            pf_gui_1.show_particles(pf.particles)
            pf_gui_1.show_mean(m_x, m_y, m_h, m_confident)
            pf_gui_1.show_camera_image(camera_image)
            pf_gui_1.updated.set()
        # print('currently predicting at ', m_x, m_y, ' heading:', m_h)
        print('Current Prediction - x: {0:.2f}, y: {1:.2f}, h: {2:.2f}'.format(m_x, m_y, m_h)) 
        # TODO: Confirm that chaning the rotation algo works
        if len(markers) == 0:
            large_turn = robot.behavior.turn_in_place(degrees(46))
            large_turn.result()        
        else:
            small_turn = robot.behavior.turn_in_place(degrees(27))
            small_turn.result()
        time.sleep(2)
        if t > max_pf_iterations:
            m_confident = True
        if m_confident:
            robot.motors.set_wheel_motors(0, 0)
            print('Confident')
            converged_statement = robot.behavior.say_text("Particle Filter has converged")
            converged_statement.result()            
    return m_x, m_y, m_h

def rrt_move_to(robot, start_x, start_y, start_h, end_x, end_y):    
    start_point = Node((start_x, start_y))
    end_goal = Node((end_x, end_y))
    cmap.clear_goals()
    cmap.reset_paths()
    cmap.add_goal(end_goal)
    cmap.set_start(start_point)
    print('Starting RRT - x: {0:.2f}, y: {1:.2f}, h: {2:.2f}'.format(start_point.x, start_point.y, start_h)) 
    RRT(cmap, cmap.get_start())
    path = cmap.get_smooth_path()
    marked = {}
    next_node = 1
    robot.behavior.set_head_angle(MIN_HEAD_ANGLE)
    vector_angle = start_h
    vector_pos = start_point
    while vector_pos not in cmap.get_goals():
        next_pos = path.pop(0)
        #print('x: {0:.2f}, y: {1:.2f}'.format(next_pos.x, next_pos.y))
        angle = np.arctan2(next_pos.y - vector_pos.y, next_pos.x - vector_pos.x)        
        if abs(angle - vector_angle) > 0.01:
                turn_node = robot.behavior.turn_in_place(anki_vector.util.Angle(radians=angle - vector_angle))
                turn_node.result()
                #print('Had to turn')
        #print('Driving to next node in path')
        straight_move = robot.behavior.drive_straight(distance_mm(get_dist(vector_pos, next_pos)),speed_mmps(200))
        straight_move.result()
        vector_pos = next_pos
        vector_angle = angle
        cmap.set_start(vector_pos)
    print('Reached goal - x: {0:.2f}, y: {1:.2f}, h: {2:.2f}'.format(vector_pos.x, vector_pos.y, vector_angle)) #TODO : Confirm this works 
    return vector_pos, vector_angle

def pick_up_cube(robot):
    connect_response = robot.world.connect_cube() #TODO: Might error on multiple runs
    light_cube = robot.world.connected_light_cube
    while light_cube is None:
        time.sleep(0.1)
        print('Still connecting')
        light_cube = robot.world.connected_light_cube
    print('Connected')
    print(connect_response.result())
    #print(connect_response.result().status)
    time.sleep(1)
    print(light_cube.is_visible, 'Vis')
    flag = True
    curr = light_cube.is_visible
    while curr is False:
        turn = robot.behavior.turn_in_place(degrees(45))
        turn.result()        
        time.sleep(2)
        curr = light_cube.is_visible    
    print('im out', curr)
    pick_up = robot.behavior.pickup_object(light_cube, use_pre_dock_pose = True)
    pick_up.result()
    #pick_up = robot.behavior.set_lift_height(1.0)
    #pick_up.result()

def saving_curr_pose(robot):
    #lift_up = robot.behavior.set_lift_height(1.0)
    #lift_up.result()
    prev_pose = robot.pose
    #put_down = robot.behavior.set_lift_height(0.0)
    #put_down.result()
    return prev_pose

def run():
    args = anki_vector.util.parse_command_args()    
    with anki_vector.AsyncRobot(serial=args.serial) as robot:
        ### PF Converging
        start_x, start_y, start_h = run_particle_filter(robot)
        start_x, start_y, start_h = (scale_factor * start_x,scale_factor* start_y,math.radians(start_h))
        i = 0        
        ### Go to pick up zone
        # goal_x, goal_y = (8.5 * scale_factor,9.5 * scale_factor)         
        while True:
            #print('Starting from - x: {0:.2f}, y: {1:.2f}, h: {2:.2f}'.format(start_x, start_y, start_h)) #TODO : Confirm this works 
            end_pos, end_ang = rrt_move_to(robot, start_x,start_y,start_h,PICKUP_LOC.x,PICKUP_LOC.y)        
            end_pos, end_ang = rrt_move_to(robot, PICKUP_LOC.x,PICKUP_LOC.y,end_ang,PICKUP_LOC.x,PICKUP_LOC.y + 1)        

            if i == 0: #once done localized and starting to pick stuff up
                cmap.add_obstacle(obstacle_nodes)
                i+=1 
            ### Pick up cube
            thing = robot.behavior.say_text("ready to begin delivery")
            thing.result()
            prev_pose = saving_curr_pose(robot)        
            pick_up_cube(robot)
            move_to_prev_pose = robot.behavior.go_to_pose(prev_pose)
            move_to_prev_pose.result()
            print('returned to previous pose')

            ### Go to drop off 
            end_pos, end_ang = rrt_move_to(robot, end_pos.x, end_pos.y - 0.3 * scale_factor,end_ang,STORAGE_LOC.x,STORAGE_LOC.y)        
            #end_pos, end_ang = rrt_move_to(robot, end_pos.x, end_pos.y,end_ang,STORAGE_LOC.x,STORAGE_LOC.y)        
            print('placing down now')        
        #drop = robot.behavior.set_lift_height(0.0)
        #drop.result()
            drop = robot.behavior.place_object_on_ground_here()
            drop.result()
            #prev_pose = saving_curr_pose(robot)
            #move_to_prev_pose = robot.behavior.go_to_pose(prev_pose)
            #move_to_prev_pose.result()
            #start_x, start_y, start_h = (end_pos.x - 0.15 * scale_factor, end_pos.y - 1.20 * scale_factor, end_ang)
            #start_x, start_y, start_h = (end_pos.x, end_pos.y, end_ang)
            #cmap.clear_obstacles()
            #end_pos, end_ang = rrt_move_to(robot, start_x,start_y,start_h, 13*25,9*25)  
            #print("Expected_x: ", end_pos.x,"\nExpected_y: ", end_pos.y, "\nExpected Angle: ", end_ang)
            #time.sleep(10000)
            #dif is 1.5 inch y and .5 x
            #start_x, start_y, start_h = (end_pos.x - 0.15 * scale_factor, end_pos.y - 1.20 * scale_factor, end_ang)
            start_x, start_y, start_h = (end_pos.x - 0.7 * scale_factor, end_pos.y - 0.6 * scale_factor, end_ang)
            #start_x, start_y, start_h = (end_pos.x, end_pos.y, end_ang)
            


class VectorThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        run() 
        #print('entered')



if __name__ == '__main__':
    # robot thread
    vector_thread = VectorThread()
    vector_thread.start()

    # rrt GUI
    if rrt_gui_show:
        stopevent = threading.Event()
        visualizer = Visualizer(cmap)
        visualizer.start()
        stopevent.set()

    # pf GUI    
    if pf_gui_1_show:
        pf_gui_1.start()
