from grid import CozGrid
from particle import Particle
from utils import grid_distance, rotate_point, diff_heading_deg, add_odometry_noise, add_gaussian_noise
import setting
import math
import numpy as np

import time

#Written by Prathic Sundararajan (psundararajan3) and Marc Hoeltge (mhoeltge3)

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """    
    motion_particles = []
    for index, particle in enumerate(particles): 
        curr_x, curr_y, curr_h = particle.xyh
        curr_x2, curr_y2 = rotate_point(odom[0], odom[1], particle.xyh[2])
        curr_x += curr_x2
        curr_y += curr_y2
        curr_h += odom[2]
        pass_in = [curr_x, curr_y, curr_h]
        curr_x, curr_y, curr_h = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        """
        if odom[0] != 0.0 and odom[1] != 0 and odom[2] != 0: #moves and turns
                curr_x, curr_y, curr_h = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        elif odom[2] != 0.0 and odom[0] == 0 and odom[1] == 0: #if only turns
                _, _, curr_h = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        elif odom[2] == 0.0 and (odom[0] != 0 or odom[1] != 0): #if moves x or y
                curr_x, curr_y, _ = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        """
        #motion_particles[index].x = curr_x
        #motion_particles[index].y = curr_y
        #motion_particles[index].h = curr_h
        motion_particles.append(Particle(curr_x, curr_y, curr_h))
        #time.sleep(1)
        #particle.x += odom[0]
        #particle.y += odom[1]         
    return motion_particles    
# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
                            curr_marker_pairings = []        
    """
    measured_particles = particles
    particle_weight = []    
    tot_robot_markers = len(measured_marker_list)
    robot_marker_dist_list = []
    for index, particle in enumerate(measured_particles):
        if not measured_marker_list:
            particle_weight.append(1/setting.PARTICLE_COUNT)
            continue
        simulated_marker_list = particle.read_markers(grid)            
        tot_particle_markers = len(simulated_marker_list)
        matched_robot_markers = []
        matched_particle_markers = []
        #print("Number of markers by particle",tot_particle_markers)
        for l, curr_robot_marker in enumerate(measured_marker_list):
            if len(simulated_marker_list) == 0:
                break
            #do we need to add noise here?
            min_particle = simulated_marker_list[0]
            closest_dist = 0 #might cause problems later is this isn't reset            
            for j, curr_particle_marker in enumerate(simulated_marker_list):
                distance_to_robot_marker = grid_distance(curr_robot_marker[0], curr_robot_marker[1], curr_particle_marker[0], curr_particle_marker[1])
                if distance_to_robot_marker < closest_dist:
                    closest_dist = distance_to_robot_marker
                    min_particle = simulated_marker_list[j]
            matched_robot_markers.append(curr_robot_marker)
            matched_particle_markers.append(min_particle)
            simulated_marker_list.remove(min_particle)
        prob = 1.0
        #need to figure out which pairs are related 
        biggest_angle_component = (45 ** 2) / (2 * (setting.MARKER_ROT_SIGMA**2))
        biggest_distance_component = 0
        for j in range(len(matched_robot_markers)):
            curr_sim_marker = matched_particle_markers[j]
            curr_rob_marker = matched_robot_markers[j]
            d = grid_distance(curr_sim_marker[0], curr_sim_marker[1], curr_rob_marker[0], curr_rob_marker[1])
            ang = diff_heading_deg(curr_sim_marker[2], curr_rob_marker[2])    
            #print('dist',d)
            #print('ang',ang)
            #print('sigma',setting.MARKER_TRANS_SIGMA)   
            biggest_distance_component= max((d**2)/(2*setting.MARKER_TRANS_SIGMA**2), biggest_distance_component) #TODO: potentially switch to a better method         
            guassian_density_function = math.exp(-1 * ((d**2)/(2*setting.MARKER_TRANS_SIGMA**2) + (ang**2)/(2*setting.MARKER_ROT_SIGMA**2)))
            #print('dumbresult',guassian_density_function)
            prob = prob * guassian_density_function
        if tot_particle_markers is not tot_robot_markers:
            for i in range(abs(tot_particle_markers - tot_robot_markers)):
                prob = prob * math.exp(-1 *biggest_distance_component - biggest_angle_component)
        if not grid.is_in(particle.x, particle.y):
            #print('not in grid')
            prob = 0
        particle_weight.append(prob)        
    #normalizing    
    sum1 = np.sum(particle_weight)
    #print(particle_weight)
    #print(sum1)
    norm_weights = particle_weight / sum1
    #resampling
    #print(norm_weights)
    NUM_TO_REMOVE = 50
    random_selection = np.random.choice(particles, len(particles) - NUM_TO_REMOVE, replace=True, p=norm_weights)
    #print(random_selection)
    resampled_particles = []
    for chosen_particle in random_selection:
        newParticle_x, newParticle_y, newParticle_h = add_odometry_noise(chosen_particle.xyh, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        newParticle = Particle(newParticle_x, newParticle_y, newParticle_h)
        resampled_particles.append(newParticle)
    rand_particles = Particle.create_random(setting.PARTICLE_COUNT - len(resampled_particles), grid)  
    for i in range(setting.PARTICLE_COUNT - len(resampled_particles)):
        resampled_particles.append(rand_particles[i])
    #print(len(particles))
    return resampled_particles

    """   
     #move particle equal to amount that robot reported
        curr_x, curr_y, curr_h = particle.xyh
        curr_x += odom[0]
        curr_y += odom[1]
        #curr_h += odom[2]
        curr_x, curr_y = rotate_point(curr_x, curr_y, particle.xyh[2])
        pass_in = [curr_x, curr_y, curr_h]
        if odom[0] != 0.0 and odom[1] != 0 and odom[2] != 0: #moves and turns
                curr_x, curr_y, curr_h = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        elif odom[2] != 0.0 and odom[0] == 0 and odom[1] == 0: #if only turns
                _, _, curr_h = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        elif odom[2] == 0.0 and (odom[0] != 0 or odom[1] != 0): #if moves x or y
                curr_x, curr_y, _ = add_odometry_noise(pass_in, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        #motion_particles[index].x = curr_x
        #motion_particles[index].y = curr_y
        #motion_particles[index].h = curr_h
        motion_particles.append(Particle(curr_x, curr_y, curr_h))
    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    time.sleep(1)

    #cut out bottom x% of particles
    removed_particles_percentage = 10 #TODO: Potentially switch to 40-50 particles
    removed_num_particles = int(setting.PARTICLE_COUNT / removed_particles_percentage)
    #sorted_resampled_particles = np.sort(resampled_particles)
    new_sorted_weights = norm_weights[random_selection]
    #new_sorted_weights = np.sort(new_sorted_weights)
    new_sorted_weights_index = np.argsort(new_sorted_weights)
    #print(new_sorted_weights_index)
    sorted_resampled_particles = resampled_particles
    for i, new_index in enumerate(new_sorted_weights_index):
        sorted_resampled_particles[i] = resampled_particles[new_index] 
    rand_particles = Particle.create_random(removed_num_particles, grid)   
    for i in range(removed_num_particles):        
        sorted_resampled_particles[i] = rand_particles[i]

                    Robot: [A, B, C]
                marker_distances: 
                A->B: 4
                A->C: 7
                B->C: 2
                [4,7,2]
            Simulated: [B, C, A]
                sim_marker_distances:
                B->A: 4
                d->A: 6
                C->A: 7
                [8,6,7]

                robot:  [1,3,4]
                particle: [3,4]
            Cases: All same markers and all are found
            Some markers are in particle but not robot
            Vice versa 
            No markers in robot but some in particles
            Vice versa                              
            #for i, sim_marker in enumerate(simulated_marker_list):
            #if len(measured_marker_list) < len(simulated_marker_list):            
            #util.grid_distance(x1, y1, x2, y2)
            #util.diff_heading_deg(heading1, heading2)
            ourOldApproach
             for i in range(len(measured_marker_list)):
        for j in range(i+1, len(measured_marker_list)):
            robot_marker_dist_list.append(grid_distance(measured_marker_list[i][0], measured_marker_list[i][1], measured_marker_list[j][0], measured_marker_list[j][1]))
    sorted(robot_marker_dist_list)
                    simulated_marker_dist_list = []
                    
        for i in range(len(simulated_marker_list)):
            for j in range(i+1, len(simulated_marker_list)):
                simulated_marker_dist_list.append(grid_distance(simulated_marker_list[i][0], simulated_marker_list[i][1], simulated_marker_list[j][0],simulated_marker_list[j][1]))          
        sorted(simulated_marker_dist_list)                
        min_num_marker = np.minimum(tot_robot_markers, tot_particle_markers)  
        for i in range(min_num_marker):
    """
    
    
































































