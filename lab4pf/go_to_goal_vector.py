#Marc Hoeltge and Prathic Sundarajan

## If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
## try uncommenting the following snippet

try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass

from skimage import color
import anki_vector
from anki_vector.events import Events
from anki_vector import annotate, events
from anki_vector.util import degrees, distance_mm, distance_inches, speed_mmps
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image
from tkinter import *
import math
from lab4pf.markers import detect, annotator_vector
from lab4pf.grid import CozGrid
#from gui import GUIWindow
from lab4pf.particle import Particle, Robot
from lab4pf.setting import *
from lab4pf.particle_filter import *
#from lab4pf.pf_gui import *
from lab4pf.utils import *

import random

global flag_odom_init, last_pose
global grid, gui, pf
#particle filter functionality

class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        #print(m_confident)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
flag_odom_init = False
#last_pose = anki_vector.util.Pose(0,0,0,angle_z=anki_vector.util.Angle(degrees=0))

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "./lab4pf/map_arena.json"
grid = CozGrid(Map_filename)
#gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)


def compute_odometry(curr_pose, last_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a anki_vector.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    #global last_pose, flag_odom_init
    global flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees
    
    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Vector's camera.

    This can be called using the following:

    markers, camera_image = marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: anki_vector.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh) 
          (as expected by the particle filter's measurement update)
        - a PIL Image of what Vector's camera sees with marker annotations
    '''

    global grid

    # Wait for the latest image from Vector
    image_raw = robot.camera.latest_image.raw_image
    image = np.array(image_raw)

    # Convert the image to grayscale
    image = color.rgb2gray(image)
    
    # Detect the markers
    markers, diag = detect.detect_markers(image, camera_settings, include_diagnostics=True)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x,y,h in marker_list]

    # Annotate the camera image with the markers
    if not show_diagnostic_image:
        annotated_image = image_raw.resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator_vector.annotate_markers(annotated_image, markers, scale=2)
    else:
        diag_image = color.gray2rgb(diag['filtered_image'])
        diag_image = Image.fromarray(np.uint8(diag_image * 255)).resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator_vector.annotate_markers(diag_image, markers, scale=2)
        annotated_image = diag_image

    return marker_list, annotated_image

    
def run():

    #global flag_odom_init, last_pose
    global flag_odom_init
    global grid, pf #gui
    args = anki_vector.util.parse_command_args()

    # Default Values of camera intrinsics matrix
    camera_settings = np.array([
        [296.54,      0, 160],    # fx   0  cx
        [     0, 296.54, 120],    #  0  fy  cy
        [     0,      0,   1]     #  0   0   1
    ], dtype=np.float)


    with anki_vector.AsyncRobot(serial=args.serial) as robot:
        robot.behavior.set_head_angle(degrees(0))
        # robot.camera.image_stream_enabled = True
        # robot.camera.color_image_enabled = False
        # robot.camera.enable_auto_exposure()


        # YOUR CODE HERE

        ###################
        robot.camera.init_camera_feed()
        camera_settings = np.array([
            [296.54,      0, 160],    # fx   0  cx
            [     0, 296.54, 120],    #  0  fy  cy
            [     0,      0,   1]     #  0   0   1
        ], dtype=np.float)
        m_confident = False
        not_at_goal = True
        #particle_filter = ParticleFilter(grid)
        #image = PhotoImage(file="cs3630.gif")                        

        last_pose = robot.pose

        GOAL_X = 6
        GOAL_Y = 10
        GOAL_H = 0

        # marker_annotate = annotator_vector.MarkerAnnotator(robot.camera.image_annotator)
        # robot.camera.image_annotator.add_annotator('Marker', marker_annotate)
        while True:
            while robot.status.is_picked_up:
                print("Robot is picked up")
                m_confident = False
                not_at_goal = True
                pf = ParticleFilter(grid)
                #gui.show_particles(pf.particles)
                #gui.show_mean(m_x, m_y, m_h, m_confident)
                #gui.updated.set()


            #if not m_confident:
                #robot.motors.set_wheel_motors(16,3)
            t = 0
            while not m_confident:
                t += 1
                time.sleep(1.2)
                dx, dy, diff_heading_deg = compute_odometry(robot.pose, last_pose) #odometery 
                markers, camera_image = marker_processing(robot, camera_settings, show_diagnostic_image=False)  
                print("Run", t, len(markers))
                m_x, m_y, m_h, m_confident = pf.update((dx, dy, diff_heading_deg), markers)
                #gui.update()
                last_pose = robot.pose
                gui.show_particles(pf.particles)
                gui.show_mean(m_x, m_y, m_h, m_confident)
                #gui.show_robot(robot)
                gui.show_camera_image(camera_image)
                gui.updated.set()
                
                
                if len(markers) == 0:
                    robot.behavior.turn_in_place(degrees(46))
                else:
                    robot.behavior.turn_in_place(degrees(27)) 
                if t == 60:
                    m_confident = True
                if m_confident:
                    print('confident')
                    robot.behavior.say_text("Particle Filter has converged")
                    time.sleep(3)
                    #robot.motors.set_wheel_motors(0,0)

            
            if not_at_goal:
                print("Calculating movement to goal")
                dist_x = abs(GOAL_X - m_x)
                dist_y = abs(GOAL_Y - m_y)
                
                pythagorean_distance = math.sqrt(dist_x**2 + dist_y**2)
                '''
                #Q1 - math.degrees(math.atan(dist_y/dist_x))
                #Q2 - 180 - math.degrees(math.atan(dist_y/dist_x))
                #Q3 - 180 + math.degrees(math.atan(dist_y/dist_x))
                #Q4 - 360 - math.degrees(math.atan(dist_y/dist_x))
                '''
                print(math.degrees(math.atan(dist_y/dist_x)), '\ndisty', dist_y, '\ndistx', dist_x, '\nmx', m_x, '\my', m_y, '\nmh', m_h)
                if GOAL_X - m_x > 0 and GOAL_Y - m_y > 0:
                    print(1)
                    dist_ang =  math.degrees(math.atan(dist_y/dist_x)) - m_h
                elif GOAL_X - m_x < 0 and GOAL_Y - m_y > 0:
                    print(2)
                    dist_ang = 180 - m_h - math.degrees(math.atan(dist_y/dist_x))
                elif GOAL_X - m_x < 0 and GOAL_Y - m_y < 0:
                    print(3)
                    dist_ang = 180 - m_h + math.degrees(math.atan(dist_y/dist_x))
                elif GOAL_X - m_x > 0 and GOAL_Y - m_y < 0:
                    print(4)
                    dist_ang = 360 -m_h - math.degrees(math.atan(dist_y/dist_x))
                else: 
                    dist_ang = 0
                print("turning in place now", degrees(dist_ang), dist_ang)
                #robot.behavior.say_text("Turning", degrees(dist_ang))
                robot_turn = robot.behavior.turn_in_place(degrees(dist_ang))
                while not robot_turn.done():
                    time.sleep(.1)
                    print("Stuck Turning")
#               time.sleep(6)
                #robot.behavior.say_text("Moving to goal")
                print("Moving straight towards goal now")
                goal_straight = robot.behavior.drive_straight(distance_mm(pythagorean_distance * 25.4),speed_mmps(100))
                print(goal_straight.done())
                while not goal_straight.done():
                    time.sleep(.5)
                print("To do last turn towards 0 degrees", 360 - (dist_ang+m_h))
                time.sleep(3)
                robot_turn = robot.behavior.turn_in_place(degrees(360 - (dist_ang+m_h)))
                while not robot_turn.done():
                    time.sleep(.1)
                    print("Stuck Turning")
                not_at_goal = False
                #robot.behavior.say_text("Reached Goal")                
                time.sleep(2)
                print("Playing Animation Trigger 1:")
                robot.anim.play_animation_trigger('GreetAfterLongTime')
                print("COMPLETED")
                




        ###################
    
class VectorThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        run()        

if __name__ == '__main__':

    # vector thread
    vector_thread = VectorThread()
    vector_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.start()
    gui.start()
