"""
Pre-Set 
#1 generate the obstacle for fragile
Expected Work Flow
#1 Need to do localization based off the markers
#2 Drives to pick up zone
#3 Gives audio cue ready for pick up 
#4 Picks up and takes it to drop off zone 
#5 Returns Back to pick up zone and repeat 
"""
from skimage import color
import anki_vector
from anki_vector.events import Events
from anki_vector import annotate, events
from anki_vector.behavior import MIN_HEAD_ANGLE, MAX_HEAD_ANGLE, MAX_LIFT_HEIGHT, MIN_LIFT_HEIGHT
from anki_vector.util import degrees, distance_mm, distance_inches, speed_mmps, Angle, Pose
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image
import math
import random

#LAB4 STUFF
#from lab4pf.grid import CozGrid
from lab4pf.grid import CozGrid
from lab4pf.gui import GUIWindow
from lab4pf.particle_filter import *
from lab4pf.particle import Particle, Robot
from lab4pf.utils import *
from lab4pf.go_to_goal_vector import ParticleFilter, compute_odometry, marker_processing
import math
import time

#LAB5 STUFF
from lab5rrt.rrt_vector import step_from_to, node_generator, RRT, get_global_node, detect_cube_and_update_cmap
from lab5rrt.cmap import *
from lab5rrt.utils import *
from lab5rrt.gui import Visualizer


def run_particle_filter(robot):
    # map
    global gui, grid, pf
    print("Entering run pf method")
    #Map_filename = "./lab4pf/map_arena.json"
    #grid = CozGrid(Map_filename)
    print('running pf')

    robot.behavior.set_head_angle(degrees(0))
    # robot.camera.image_stream_enabled = True
    # robot.camera.color_image_enabled = False
    # robot.camera.enable_auto_exposure()

    # YOUR CODE HERE

    ###################
    robot.camera.init_camera_feed()
    camera_settings = np.array([
        [296.54,      0, 160],    # fx   0  cx
        [0, 296.54, 120],  # 0  fy  cy
        [0,      0,   1]  # 0   0   1
    ], dtype=np.float)
    m_confident = False
    not_at_goal = True
    image_raw = robot.camera.latest_image.raw_image

    #particle_filter = ParticleFilter(grid)
    #image = PhotoImage(file="cs3630.gif")

    last_pose = robot.pose

    t = 0
    m_x, m_y, m_h = (0, 0, 0)
    while not m_confident:
        t += 1
        #time.sleep(1.2)
        dx, dy, diff_heading_deg = compute_odometry(
            robot.pose, last_pose)  # odometery
        if t == 1:
            length = 0
        else:
            prev_markers = markers
            length = len(prev_markers)
        markers, camera_image = marker_processing(
            robot, camera_settings, show_diagnostic_image=False)
        print("Run", t, len(markers))
        m_x, m_y, m_h, m_confident = pf.update(
            (dx, dy, diff_heading_deg), markers)
        #gui.update()
        last_pose = robot.pose
        #gui.show_particles(pf.particles)
        #gui.show_mean(m_x, m_y, m_h, m_confident)
        #gui.show_robot(robot)
        #gui.show_camera_image(camera_image)
        #gui.updated.set()
        print('confident at ', m_x, m_y, ' heading:', m_h)

        if True:
            robot_turn = robot.behavior.turn_in_place(degrees(45))
            while not robot_turn.done():
                time.sleep(.05)
                #print('turning')
            time.sleep(.5)
        elif len(markers) == 0:
            curr = random.random() * 10
            robot_turn = robot.behavior.turn_in_place(
                degrees(40 + curr))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        elif length == 0 & len(markers) != 0:
            curr = random.random() * 10
            robot_turn = robot.behavior.turn_in_place(degrees(5 + curr))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        else:
            robot_turn = robot.behavior.turn_in_place(degrees(35))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        if t == 30:
            m_confident = True
        if m_confident:
            robot.motors.set_wheel_motors(0, 0)
            print('confident')
            robot.behavior.say_text("Particle Filter has converged")
            time.sleep(3)
            #robot.motors.set_wheel_motors(0,0)
    #m_x, m_y, m_h = (6, 4, 0)
    return m_x, m_y, m_h

    '''
    while not m_confident:
        t += 1
        time.sleep(1.2)
        dx, dy, diff_heading_deg = compute_odometry(robot.pose, last_pose) #odometery 
        markers, camera_image = marker_processing(robot, camera_settings, show_diagnostic_image=False)  
        print('Run', t, len(markers), '(' + str(m_x) + ', ' + str(m_y) + ', ' + str(m_h)+ ')')
        m_x, m_y, m_h, m_confident = pf.update((dx, dy, diff_heading_deg), markers)
        #gui.update()
        last_pose = robot.pose
        #gui.show_particles(pf.particles)
        #gui.show_mean(m_x, m_y, m_h, m_confident)
        #gui.show_robot(robot)
        #gui.show_camera_image(camera_image)
        #gui.updated.set()
        
        if len(markers) == 0:
            robot.behavior.turn_in_place(degrees(46))
        else:
            robot.behavior.turn_in_place(degrees(27)) 
        if t == 50:
            m_confident = True
        if m_confident:
            print('confident at ', m_x, m_y, ' heading:', m_h)
            #gui.update()
            robot.behavior.say_text("Particle Filter has converged at ")
            #robot.behavior.turn_in_place(360 - m_h)
            """
            m_x, m_y = (13, 9)
            print("About to hard coreturn ")
            robot.behavior.turn_in_place((degrees(290)))
            time.sleep(6)
            print("About to turn usign pose")
            pose = Pose(x=m_x, y=m_y, z=0, angle_z=Angle(degrees=0))
            pose_future = robot.behavior.go_to_pose(pose)
            time.sleep(8)
            pose_future.cancel()
            print("Done sleeping")
            # don't need to do this because we are returning heading as well
            """
            break
            #robot.motors.set_wheel_motors(0,0)
    '''

    #if not m_confident:
    #    robot.motors.set_wheel_motors(18,4)
    t = 0
    while not m_confident:
        t += 1
        #time.sleep(1.2)
        dx, dy, diff_heading_deg = compute_odometry(
            robot.pose, last_pose)  # odometery
        if t == 1:
            length = 0
        else:
            prev_markers = markers
            length = len(prev_markers)
        markers, camera_image = marker_processing(
            robot, camera_settings, show_diagnostic_image=False)
        print("Run", t, len(markers))
        m_x, m_y, m_h, m_confident = pf.update(
            (dx, dy, diff_heading_deg), markers)
        #gui.update()
        last_pose = robot.pose
        #gui.show_particles(pf.particles)
        #gui.show_mean(m_x, m_y, m_h, m_confident)
        #gui.show_robot(robot)
        #gui.show_camera_image(camera_image)
        #gui.updated.set()
        print('confident at ', m_x, m_y, ' heading:', m_h)

        if len(markers) == 0:
            curr = random.random() * 10
            robot_turn = robot.behavior.turn_in_place(
                degrees(40 + curr))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        elif length == 0 & len(markers) != 0:
            curr = random.random() * 10
            robot_turn = robot.behavior.turn_in_place(degrees(5 + curr))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        else:
            robot_turn = robot.behavior.turn_in_place(degrees(35))
            while not robot_turn.done():
                time.sleep(.05)
                #print("Stuck Turning")
            time.sleep(0.5)
        if t == 50:
            m_confident = True
        if m_confident:
            robot.motors.set_wheel_motors(0, 0)
            print('confident')
            robot.behavior.say_text("Particle Filter has converged")
            time.sleep(3)
            #robot.motors.set_wheel_motors(0,0)

    return m_x, m_y, m_h

    '''
        current position is m_x, m_y, m_h
    '''





def units_to_grid(a):
    return int(a * 25)


class RobotThread(threading.Thread):
    """Thread to run vector code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)
    def plan_route(self, robot, startx, starty, starth):
        global cmap, stopevent        
        RRT(cmap, cmap.get_start())
        path = cmap.get_smooth_path()

        marked = {}
        update_cmap = False
        next_node = 1

        robot.behavior.set_head_angle(MIN_HEAD_ANGLE)

        prev_angle = starth
        while len(path) > next_node:
            if path is None or path == []:
                print('no path found')
                break

            if (len(path) > next_node):
                move_to = path[next_node]
                robot_curr_x = path
                move_to = path[next_node]
                robot_curr_x = path[next_node - 1].x
                robot_curr_y = path[next_node - 1].y
                #robot_curr_z  = robot.pose.position.z
                #end_angle = math.atan((move_to.y - robot_curr_y)/(move_to.x - robot_curr_x))
                end_angle = math.atan2(
                    move_to.y - robot_curr_y, move_to.x - robot_curr_x)  # x  ç≈TODO: UNSURE
                dist_to_go = get_dist(move_to, Node(
                    ((robot_curr_x, robot_curr_y))))
                print('Turning to next node')
                end_angle *= 57.29
                print('end_angle', end_angle)
                print('prev_angle', prev_angle)
                print(degrees(end_angle - prev_angle),
                    "Turn needed from pf converge")
                turn = robot.behavior.turn_in_place(
                    degrees(end_angle - prev_angle))
                turn.result()
                while turn.done() is False:                
                    print('waiting for turn')
                    time.sleep(0.1)            
                prev_angle = end_angle
                time.sleep(2)
                print('finished turning')
                print('Driving to next node')
                drive = robot.behavior.drive_straight(
                    distance_mm(dist_to_go), speed_mmps(50))
                drive.result()
                time.sleep(1)
                # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle
                vector_pos = path[next_node]

                # Set new start position for replanning with RRT
                cmap.set_start(vector_pos)

            #detect any visible obstacle cubes and update cmap
            # https://gatech.bluejeans.com/3660959485
            #print(len(path), "Total Nodes")
            #print(next_node - 1, 'curr node')
            #print(path[next_node - 1].x, path[next_node - 1].y)
            update_cmap = False
            #update_cmap, goal_center, marked = detect_cube_and_update_cmap(robot, marked,  Node(((path[next_node].x, path[next_node].y))), cmap)
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

        """#instructor solution
        RRT(cmap, cmap.get_start())
        path = cmap.get_smooth_path()
        marked = {}
        update_cmap = False
        next_node = 1
        robot.behavior.set_head_angle(MIN_HEAD_ANGLE)
        vector_pos = Node((startx, starty))
        vector_angle = starth * .01745329252
        while vector_pos not in cmap.get_goals():

            #break if path is none or empty, indicating no path was found
            if (path is None or len(path) == 0):
                print("path is none")  # sanmesh
                break

            # Get the next node from the path
            #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
            #you can calculate the angle to turn through a trigonometric function
            next_pos = path.pop(0)
            print('x: {0:.2f}, y: {1:.2f}'.format(next_pos.x, next_pos.y))
            angle = np.arctan2(next_pos.y - vector_pos.y,
                                next_pos.x - vector_pos.x)
            print("driving robot to next node in path")
            if abs(angle - vector_angle) > 0.01:
                robot.behavior.turn_in_place(
                    anki_vector.util.Angle(radians=angle - vector_angle))
            robot.behavior.drive_straight(distance_mm(
                get_dist(vector_pos, next_pos)*25), speed_mmps(30))
            # Update the current vector position (vector_pos and vector_angle) to be new node position and angle
            vector_pos = next_pos
            vector_angle = angle

            # Set new start position for replanning with RRT
            cmap.set_start(vector_pos)

            #detect any visible obstacle cubes and update cmap
            #print("detect_cube_and_update_cmap")
            #update_cmap, goal_center, marked = detect_cube_and_update_cmap(
            #    robot, marked, vector_pos, cmap)

            #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths
            if update_cmap:
                # Found the goal
                cmap.reset_paths()
                RRT(cmap, cmap.get_start())
                path = cmap.get_smooth_path()
        robot_curr_x, robot_curr_y, end_angle = vector_pos.x, vector_pos.y, vector_angle
        """

        print('reached goal ' + str(robot_curr_x) + ', ' +
            str(robot_curr_y) + ', ' + str(end_angle))
        return robot_curr_x, robot_curr_y, end_angle

    def picking_up_cube(self, robot, connection):
        #time.sleep(7)
        light_cube = robot.world.connected_light_cube
        curr_loop = 0
        while light_cube is None:
            time.sleep(0.1)
            print('Connecting to light cube')
            #print(connection.status, 'is cube connected')
            curr_loop += 1
            if curr_loop % 100 == 0:
                t2 = robot.behavior.turn_in_place(degrees(25))
                while t2.done() is False:
                    time.sleep(0.1)
                    print('waiting for turn to finish')
            elif curr_loop % 300 == 0:
                robot.world.connect_cube()
            light_cube = robot.world.connected_light_cube
        #t5.result()
        print(light_cube.is_connected, 'cube is connected')
        if light_cube:
            thing = robot.behavior.say_text("ready to begin delivery")
            #say = robot.behavior.say_text('ready to begin delivery')
            #while not say.done():
            #    time.sleep(.1)
            t1 = robot.behavior.dock_with_cube(light_cube)
            t1.result(timeout= None)
            while t1.done() is False:
                time.sleep(0.1)
                print('waiting for docking to finish')
            print(t1.done(), 'docking done now')
            #thing = robot.behavior.say_text("Done docking")
            #say = robot.behavior.say_text('ready to begin delivery')
            #while not say.done():
            #    time.sleep(.1)
            cmap.add_obstacle([Node((263, 450)), Node((338,450)), Node((338,163)), Node((263, 163))])
            light_cube = robot.world.connected_light_cube
            #t2 = robot.behavior.pickup_object(light_cube)
            t2 = robot.behavior.set_lift_height(1.0)
            while t2.done() is False:
                time.sleep(0.1)
                print('waiting for pickup to finish')

    def run(self):
        global cmap, stopevent
        # Please refrain from enabling use_viewer since it uses tk, which must
        # be in main thread
        args = anki_vector.util.parse_command_args()
        t = abs(2-1)
        # https://www.dropbox.com/t/rUlrxh2cZ0B6jJAM
        with anki_vector.AsyncRobot(serial=args.serial) as robot:
            #robot.world.disconnect_cube()
            # connection = robot.world.connect_cube()
            #z = 0
            #while z <= 15:
            #    done = robot.behavior.turn_in_place(degrees(45))
            #    done.result()
            ##    print('Particle Filter converging')
             #   time.sleep(.01)
             #   z= z + 1
            #robot.behavior.turn_in_place(degrees(90 - endh))
            startx, starty, starth = run_particle_filter(robot)
            #(4, 6, 0)
            #
            #(4, 6, 0)
            #run_particle_filter(robot)
            #cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)
            cmap_width, cmap_height = cmap.get_size()
            pickup_location = Node((units_to_grid(3), units_to_grid(8)))
            storage_location = Node((units_to_grid(20), units_to_grid(13)))
            cmap.add_goal(pickup_location)
            cmap.add_obstacle([Node((263, 450)), Node((338,450)), Node((338,163)), Node((263, 163))])
            cmap.reset_paths()
            cmap.set_start(Node((units_to_grid(startx), units_to_grid(starty))))
            print('cmap start: (' + str(cmap.get_start().x) +
                  ',' + str(cmap.get_start().y) + ')')
            endx, endy, endh = self.plan_route(robot, startx, starty, starth)

            #robot.world.connect_cube()

            #while True:
            for _ in range(1):
                connection = robot.world.connect_cube()  # TODO: NEED THIS
                cmap.clear_goals()
                cmap.reset_paths()
                cmap.add_goal(Node((endx, endy + .1)))
                cmap.set_start(Node((endx, endy)))
                endx, endy, endh = self.plan_route(robot, endx, endy, endh)
                #print(degrees(end_angle - prev_angle), "Turn needed from pf converge")
                #robot.behavior.turn_in_place(degrees(90 - endh))
                time.sleep(2)
                #endh = 90
                #start pickup loop
                
                t2 = robot.behavior.set_lift_height(1.0)
                
                while t2.done() is False:
                    time.sleep(0.1)
                    print('waiting for lift to finish putting up')  
                
                pose_before_pickup = robot.pose
                t2 = robot.behavior.set_lift_height(0.0)
                while t2.done() is False:
                    time.sleep(0.1)
                    print('waiting for lift to finish puttig down')  
                #state 1, needs to pick up cube
                """
                while not robot.world.connected_light_cube:
                    time.sleep(.05)
                print('robot going to pick up cube')

                picking_up = robot.behavior.pickup_object(robot.world.connected_light_cube)
                picking_up.result()
                #time.sleep(5)
                #robot.behavior.set_lift_height(MAX_LIFT_HEIGHT)
                #time.sleep(2)
                """
                #print(connection.success, 'is cube connected')
                self.picking_up_cube(robot, connection)
                print('About to return to starting point')
                #back_to_before = robot.behavior.go_to_pose(pose_before_pickup)
                #while back_to_before.done() is False:
                #    time.sleep(0.1)
                #    print('waiting to return to starting point')   

                '''
                time.sleep(2)
                robot.behavior.dock_with_cube(robot.world.connected_light_cube)
                t2 = robot.behavior.pickup_object(robot.world.connected_light_cube)
                print(t2.done())
                #robot.behavior.set_lift_height(1.0)
                #t1 = robot.behavior.go_to_object(robot.world.connected_light_cube,  distance_mm(0))
                #t1.result()
                #print(t1.done())
                time.sleep(5)
                #robot.behavior.pickup_object(robot.world.connected_light_cube)
                #time.sleep(5)
                #robot.behavior.set_lift_height(MAX_LIFT_HEIGHT)
                #time.sleep(2)
                '''

                #robot.behavior.go_to_pose(pose_before_pickup)
                #time.sleep(3)
                print('resetting cmap')
                #cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)
                #visualizer.stop()
                #visualizer = Visualizer(cmap)
                #visualizer.start()
                cmap.clear_goals()
                cmap.clear_nodes()
                cmap.reset_paths()
                #pickup_location
                cmap.add_goal(storage_location)
                endx, endy, endh = (5, 12, 90)
                cube_loc_x,cube_loc_y  = (5, 12)
                print("THIS SHOULD BE SECOND")
                #cmap.set_start(Node((cube_loc_x, cube_loc_y)))
                #print('cmap start: (' + str(cmap.get_start().x) +
                #      ',' + str(cmap.get_start().y) + ')')
                #print('moving to drop off location')
                #endx, endy, endh = plan_route(robot, cube_loc_x, cube_loc_y,
                #90)
                cmap.clear_goals()
                cmap.reset_paths()
                cmap.add_goal(storage_location)
                cmap.set_start(Node((units_to_grid(endx), units_to_grid(endy))))     
                endx, endy, endh = self.plan_route(robot, endx, endy + 0.1, endh)                            #TODO: CHANGE BACK
                #(robot, endx, endy + .01, endh)                            
                #endx, endy, endh = (robot_curr_x, robot_curr_y, end_angle)
                #endx, endy, endh = plan_route(robot, endx, endy + .01, endh)
                #(robot, endx, endy, endh) maybe start h
                #robot.behavior.place_object_on_ground_here()
                t2 = robot.behavior.set_lift_height(0.0)
                t2.result()
                #robot.behavior.place_object_on_ground_here()
                while t2.done() is False:
                    time.sleep(0.1)
                    print('waiting to put down')
                time.sleep(5)
                """
                print('going back to pickup')
                cmap.clear_goals()
                cmap.reset_paths()
                cmap.add_goal(pickup_location)
                cmap.set_start(storage_location)
                endx, endy, endh = plan_route(robot, endx, endy, endh)
                print('Disconnecting cube')
                robot.world.disconnect_cube()
                """


if __name__ == '__main__':
    global cmap, stopevent, gui, grid, pf
    stopevent = threading.Event()
    #print(sys.argv)
    #for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
    #print(sys.argv[i])
    #if ('robot' in sys.argv[i]):
    #    robotFlag = True
    #print("Robot Flag", robotFlag)
    #creates cmap based on empty grid json
    #"start": [50, 35],
    #"goals": [] This is empty
    Map_filename = "./lab4pf/map_arena.json"
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid, show_camera=True)
    pf = ParticleFilter(grid)
    #gui.show_particles(pf)
    #gui.start()
    #gui.start()
    #cmap = CozMap("./lab5rrt/maps/emptyemptygrid.json", node_generator)
    cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    """
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    """
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()


"""
class RobotThread(threading.Thread):
    # Thread to run vector code separate from main thread
    

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        plan_route()

if __name__ == '__main__':
    global cmap, stopevent
    # vector thread
    #vector_thread = VectorThread()
    #vector_thread.start()
    #print("emtering")
    args = anki_vector.util.parse_command_args()
    # Default Values of camera intrinsics matrix
    camera_settings = np.array([
        [296.54,      0, 160],    # fx   0  cx        
        [     0, 296.54, 120],    #  0  fy  cy
        [     0,      0,   1]     #  0   0   1
    ], dtype=np.float)

    with anki_vector.AsyncRobot(serial=args.serial) as robot:
        startx, starty, starth = run_particle_filter(robot)
        cmap = CozMap("./lab5rrt/maps/emptygrid.json", node_generator)
        cmap_width, cmap_height = cmap.get_size()
        cmap.add_goal(Node((units_to_grid(5), units_to_grid(15))))
        cmap.reset_paths()
        cmap.set_start(Node((units_to_grid(startx), units_to_grid(starty))))
        print('cmap start: ' + str(cmap.get_start().x) + str(cmap.get_start().y))
        time.sleep(3)

        plan_route(robot, startx, starty, starth)


    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()

"""
