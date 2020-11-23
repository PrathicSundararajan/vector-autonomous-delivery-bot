#!/usr/bin/env python3

# Copyright (c) 2018 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tell Vector to drive up to a seen cube.

This example demonstrates Vector driving to and docking with a cube, without
picking it up.  Vector will line his arm hooks up with the cube so that they are
inserted into the cube's corners.

You must place a cube in front of Vector so that he can see it.
"""

import anki_vector
import time
from anki_vector.util import degrees, distance_mm, speed_mmps
# from anki_vector.events import Events
#from anki_vector import *

def testing_cube_pick_up(robot):
    robot.world.connect_cube()
    #time.sleep(7)
    light_cube = robot.world.connected_light_cube
    curr_loop = 0
    while light_cube is None:
        time.sleep(0.1)
        print('Connecting to light cube')
        curr_loop += 1
        if curr_loop % 50 == 0:                    
            t2 = robot.behavior.turn_in_place(degrees(25))
            while t2.done() is False:
                time.sleep(0.1)
                print('waiting for turn to finish') 
        light_cube = robot.world.connected_light_cube


    #t5.result()
    print(light_cube.is_connected, 'cube is connected')
    if light_cube:
        #thing = robot.behavior.say_text("About to dock")
        t1 = robot.behavior.dock_with_cube(light_cube, num_retries=1000)            
        #print(t1.success())
        print('reached')
        t1.result(timeout= None)
        print('reached')
        #t1.result()    
        while t1.done() is False:
            print('waiting for docking to finish')  
        #    #t1 = robot.behavior.dock_with_cube(light_cube, num_retries=5)            
            time.sleep(1)


        print(t1.done(), 'docking done now')
        #thing = robot.behavior.say_text("Done docking")
        #t2 = robot.behavior.pickup_object(light_cube, num_retries=10)
        t2 = robot.behavior.set_lift_height(1.0)
        while t2.done() is False:
            time.sleep(0.1)
            print('waiting for lift to finish')   
        turn = robot.behavior.turn_in_place(degrees(90))
        turn.result()
        while turn.done() is False:
            print('waiting for turn')
            time.sleep(0.1)
        t3 = robot.behavior.drive_straight(distance_mm(-100.2), speed_mmps(100))
        while t3.done() is False:
            time.sleep(0.1)
            print('waiting for drive') 
        t2 = robot.behavior.place_object_on_ground_here()
        while t2.done() is False:
            time.sleep(0.1)
            print('waiting to put down') 
        #time.sleep(10)
        #robot.behavior.set_lift_height(1.0)
        
        #print(t1.done())
        #t1 = robot.behavior.go_to_object(light_cube, distance_mm(0))
        #while not t1.done():
        #    time.sleep(0.1)
        #    print('waiting till we move')
        #t1.result()
        print("shoudl be by the object")
        """
        picking_up = robot.behavior.pickup_object(light_cube)
        #print(t1.done())
        #while not picking_up.done():
        #    time.sleep(0.1)
        #    print("About to wait till done picking up")
        picking_up.result()            
        """
        thing = robot.behavior.say_text("Object fucking picked up")
    else:
        print("can't connect")
    flag = False
    print('Done loop')

def testing_statement_and_rotation(robot):
    statement = robot.behavior.say_text("Particle Filter has converged")
    statement.result()
    large_turn = robot.behavior.turn_in_place(degrees(46))
    large_turn.result()
    small_turn = robot.behavior.turn_in_place(degrees(27))
    small_turn.result()

def testing_cube_pick_up_robust(robot):
    robot.behavior.set_head_angle(degrees(0))
    connect_response = robot.world.connect_cube()
    light_cube = robot.world.connected_light_cube
    prev_pose = robot.pose
    while light_cube is None:
        time.sleep(0.1)
        print('Still connecting')
        light_cube = robot.world.connected_light_cube
    print('Connected')
    print(connect_response.result())
    #print(connect_response.result().status)
    time.sleep(1)
    print(light_cube.is_visible, 'Vis')
    #moving = robot.behavior.pickup_object(light_cube, use_pre_dock_pose = True)
    #print(type(moving.result()))
    #print(moving.result().result.code)
    # 50331652 - Bad object
    # 50331683 - can't see
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
    print("Done picking up")
    back = robot.behavior.drive_straight(distance_mm(-50), speed_mmps(30))
    back.result()
    print('Starting to go back')
    backing = robot.behavior.go_to_pose(prev_pose)
    backing.result()
    print('Done going back now')
    print('placing down now')
    drop = robot.behavior.place_object_on_ground_here()
    drop.result()
    
    '''
    pick_up = robot.behavior.set_lift_height(1.0)
    pick_up.result()
    back = robot.behavior.drive_straight(distance_mm(-50), speed_mmps(30))
    back.result()
    print('placing down now')        
    drop = robot.behavior.set_lift_height(0.0)
    drop.result()
    '''


def testing_cube_pick_up_robust_old(robot):
    connect_response = robot.world.connect_cube()
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
    moving = robot.behavior.pickup_object(light_cube, use_pre_dock_pose = True)
    #print(type(moving.result()))
    print(moving.result().result.code)
    # 50331652 - Bad object
    # 50331683 - can't see
    flag = True
    curr = moving.result()
    while flag:
        if curr.result.code == 50331652:
            print('BAD OBJECT')
            turn = robot.behavior.turn_in_place(degrees(45))
            turn.result()
        elif curr.result.code == 50331683:
            print('CAN"T SEE')
            turn = robot.behavior.turn_in_place(degrees(45))
            turn.result()                    
        elif curr.result.code == 3 or curr.result.code == 0:
            print('Got there')
            print(curr.result.code)
            flag = False
        elif curr.result.code == 67108867:
            back = robot.behavior.drive_straight(distance_mm(-100), speed_mmps(30))
            back.result()
    #print(moving.result)
    #print(light_cube.pose.x)
        if flag:
            moving = robot.behavior.pickup_object(light_cube, use_pre_dock_pose = True)
            curr = moving.result() 
            time.sleep(2) 
    print('im out', curr.result)
    pick_up = robot.behavior.pickup_object(light_cube, use_pre_dock_pose = True)
    pick_up.result()
    back = robot.behavior.drive_straight(distance_mm(-50), speed_mmps(30))
    back.result()
    print('placing down now')
    drop = robot.behavior.place_object_on_ground_here()
    drop.result()
    
    '''
    pick_up = robot.behavior.set_lift_height(1.0)
    pick_up.result()
    back = robot.behavior.drive_straight(distance_mm(-50), speed_mmps(30))
    back.result()
    print('placing down now')        
    drop = robot.behavior.set_lift_height(0.0)
    drop.result()
    '''




def main():
    args = anki_vector.util.parse_command_args()
    flag = True
    docking_result = None
    with anki_vector.AsyncRobot(args.serial) as robot:
        while True:
            testing_cube_pick_up_robust(robot)
            #testing_cube_pick_up(robot)
            print('Done')
            time.sleep(100000)

            
            

            
    



if __name__ == "__main__":
    main()
