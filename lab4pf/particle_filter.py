from lab4pf.grid import *
from lab4pf.particle import Particle
from lab4pf.utils import *
import lab4pf.setting as setting
import numpy as np
np.random.seed(RANDOM_SEED)
from itertools import product

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

    for particle in particles:
        x, y, h = particle.xyh
        dx, dy, dh = odom
        c, d = rotate_point(dx, dy, h)
        nx, ny, nh = add_odometry_noise((x+c, y+d, h+dh), heading_sigma=setting.ODOM_HEAD_SIGMA, trans_sigma=setting.ODOM_TRANS_SIGMA)
        newParticle = Particle(nx, ny, nh%360)
        motion_particles.append(newParticle)

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)
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
    """
    num_random_sample = 25
    measured_particles = []
    weight = []

    if len(measured_marker_list) > 0:
        for particle in particles:
            x, y = particle.xy
            if grid.is_in(x, y) and grid.is_free(x, y):
                markers_visible_to_particle = particle.read_markers(grid)
                markers_visible_to_robot = measured_marker_list.copy()

                marker_pairs = []
                while len(markers_visible_to_particle) > 0 and len(markers_visible_to_robot) > 0:
                    all_pairs = product(markers_visible_to_particle, markers_visible_to_robot)
                    pm, rm = min(all_pairs, key=lambda p: grid_distance(p[0][0], p[0][1], p[1][0], p[1][1]))
                    marker_pairs.append((pm, rm))
                    markers_visible_to_particle.remove(pm)
                    markers_visible_to_robot.remove(rm)

                prob = 1.
                for pm, rm in marker_pairs:
                    d = grid_distance(pm[0], pm[1], rm[0], rm[1])
                    h = diff_heading_deg(pm[2], rm[2])

                    exp1 = (d**2)/(2*setting.MARKER_TRANS_SIGMA**2)
                    exp2 = (h**2)/(2*setting.MARKER_ROT_SIGMA**2)

                    likelihood = math.exp(-(exp1+exp2))
                    # The line is the key to this greedy algorithm
                    # prob *= likelihood
                    prob *= max(likelihood, setting.DETECTION_FAILURE_RATE*setting.SPURIOUS_DETECTION_RATE)

                # In this case, likelihood is automatically 0, and max(0, DETECTION_FAILURE_RATE) = DETECTION_FAILURE_RATE
                prob *= (setting.DETECTION_FAILURE_RATE**len(markers_visible_to_particle))
                # Probability for the extra robot observation to all be spurious
                prob *= (setting.SPURIOUS_DETECTION_RATE**len(markers_visible_to_robot))
                weight.append(prob)

            else:
                weight.append(0.)
    else:
        weight = [1.]*len(particles)

    norm = float(sum(weight))

    if norm != 0:
        weight = [i/norm for i in weight]
        measured_particles = Particle.create_random(num_random_sample, grid)
        measured_particles += np.random.choice(particles, setting.PARTICLE_COUNT-num_random_sample, p=weight).tolist()
    else:
        measured_particles = Particle.create_random(setting.PARTICLE_COUNT, grid)

    return measured_particles


'''
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
        motion_particles.append(Particle(curr_x, curr_y, curr_h))     
    return motion_particles  

    # ...


# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

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
        for l, curr_robot_marker in enumerate(measured_marker_list):
            if len(simulated_marker_list) == 0:
                break
            min_particle = simulated_marker_list[0]
            closest_dist = 0         
            for j, curr_particle_marker in enumerate(simulated_marker_list):
                distance_to_robot_marker = grid_distance(curr_robot_marker[0], curr_robot_marker[1], curr_particle_marker[0], curr_particle_marker[1])
                if distance_to_robot_marker < closest_dist:
                    closest_dist = distance_to_robot_marker
                    min_particle = simulated_marker_list[j]
            matched_robot_markers.append(curr_robot_marker)
            matched_particle_markers.append(min_particle)
            simulated_marker_list.remove(min_particle)
        prob = 1.0
        biggest_angle_component = (45 ** 2) / (2 * (setting.MARKER_ROT_SIGMA**2))
        biggest_distance_component = 0
        for j in range(len(matched_robot_markers)):
            curr_sim_marker = matched_particle_markers[j]
            curr_rob_marker = matched_robot_markers[j]
            d = grid_distance(curr_sim_marker[0], curr_sim_marker[1], curr_rob_marker[0], curr_rob_marker[1])
            ang = diff_heading_deg(curr_sim_marker[2], curr_rob_marker[2])       
            biggest_distance_component= max((d**2)/(2*setting.MARKER_TRANS_SIGMA**2), biggest_distance_component)          
            guassian_density_function = math.exp(-1 * ((d**2)/(2*setting.MARKER_TRANS_SIGMA**2) + (ang**2)/(2*setting.MARKER_ROT_SIGMA**2)))
            prob = prob * guassian_density_function
        if tot_particle_markers is not tot_robot_markers:
            for i in range(abs(tot_particle_markers - tot_robot_markers)):
                prob = prob * math.exp(-1 *biggest_distance_component - biggest_angle_component)
        if not grid.is_in(particle.x, particle.y):
            prob = 0
        particle_weight.append(prob)          
    sum1 = np.sum(particle_weight)
    norm_weights = particle_weight / sum1
    NUM_TO_REMOVE = 50
    random_selection = np.random.choice(particles, len(particles) - NUM_TO_REMOVE, replace=True, p=norm_weights)
    resampled_particles = []
    for chosen_particle in random_selection:
        newParticle_x, newParticle_y, newParticle_h = add_odometry_noise(chosen_particle.xyh, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        newParticle = Particle(newParticle_x, newParticle_y, newParticle_h)
        resampled_particles.append(newParticle)
    rand_particles = Particle.create_random(setting.PARTICLE_COUNT - len(resampled_particles), grid)  
    for i in range(setting.PARTICLE_COUNT - len(resampled_particles)):
        resampled_particles.append(rand_particles[i])
    return resampled_particles
'''