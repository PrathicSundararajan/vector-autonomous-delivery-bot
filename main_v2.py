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
from lab5rrt.rrt_vector import node_generator, RRT





global grid, pf_gui_1, pf, cmap 
global pf_gui_1_show, rrt_gui 
# need to have one of these showing for script to run
pf_gui_1_show = False
rrt_gui_show = True

pf_map_filename = "./lab4pf/map_arena.json"
grid = CozGrid(pf_map_filename)
pf_gui_1 = GUIWindow(grid, show_camera=True) #TODO: pf_gui might error out
cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)


def run_particle_filter(robot):
    # map
    global grid, pf, pf_gui_1
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
        print('Current Prediction - x: {0:.2f}, y: {1:.2f}, h: {2:.2f}'.format(m_x, m_y, m_h)) #TODO : Confirm this works 
        # TODO: Confirm that chaning the rotation algo works
        if len(markers) == 0:
            large_turn = robot.behavior.turn_in_place(degrees(46))
            large_turn.result()        
        else:
            small_turn = robot.behavior.turn_in_place(degrees(27))
            small_turn.result()
        time.sleep(2)
        if t == 30:
            m_confident = True
        if m_confident:
            robot.motors.set_wheel_motors(0, 0)
            print('Confident')
            converged_statement = robot.behavior.say_text("Particle Filter has converged")
            converged_statement.result()            
    return m_x, m_y, m_h

##def rrt_move_to(robot, start_x, start_y, start_h):




def run():
    # global cmap
    args = anki_vector.util.parse_command_args()    
    with anki_vector.AsyncRobot(serial=args.serial) as robot:
        ### PF Converging
        run_particle_filter(robot)

        ### Go to pick up zone

        ### Pick up cube

        ### Go to drop off 


class VectorThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        run() 
        #print('entered')



if __name__ == '__main__':
    global stopevent
    
    vector_thread = VectorThread()
    vector_thread.start()

    #rrt GUI
    if rrt_gui_show:
        stopevent = threading.Event()
        visualizer = Visualizer(cmap)
        visualizer.start()
        stopevent.set()
    print('Enttering')    
    if pf_gui_1_show:
        pf_gui_1.start()
