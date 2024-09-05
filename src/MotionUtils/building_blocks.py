from flask import config
import numpy as np
import random
from src.MotionUtils.kinematicsUtils import config_autocomplete

def sphere_collision(p1,p2,r1,r2):
    return np.linalg.norm(np.array(p1)-np.array(p2)) < r1 + r2

def max_angle_difference(conf1, conf2):
    max_difference = 0
    for angle1, angle2 in zip(conf1, conf2):
        # normalize angles to [0, 2 * np.pi] range
        angle1, angle2 = angle1 % (2 * np.pi), angle2 % (2 * np.pi)
        difference = abs(angle1 - angle2)
        # Consider the circular nature of angles
        difference = min(difference % (2 * np.pi), (2 * np.pi - difference) % (2 * np.pi))
        max_difference = max(max_difference, difference)
    return max_difference


class Building_Blocks(object):
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])

    def generate_upright_configuration(self,low, high):
        c0 = np.random.uniform(low, high)
        c1 = np.random.uniform(low, high)
        c2 = np.random.uniform(low, high)

        bias = (np.deg2rad(0), np.deg2rad(-2.5))
        c3 = -(c1 + c2) + bias[1] - np.pi

        c4 = np.random.choice([-np.pi/2, np.pi/2])

        c5 = np.pi/2

        return [c0, c1, c2, c3, c4, c5]

    def sample(self, goal_conf) -> np.array:
        if random.random() < self.p_bias:
            return np.array(goal_conf)
        else:
            conf = self.generate_upright_configuration(low=(-np.pi), high=(np.pi))
            return np.array(conf)

    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration
        """
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3], s2[0:3], self.ur_params.sphere_radius[self.ur_params.ur_links[i]], self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        # arm - obstacle collision
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obstacle in self.env.obstacles:
                    if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
                        return True
        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            if joint == self.ur_params.ur_links[0]:
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        # x - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4:
                    return True
        return False

    def local_planner(self, prev_conf, current_conf) -> bool:
        '''check for collisions between two configurations - return True if transition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        number_of_configurations_to_check = max(3, int(max_angle_difference(prev_conf, current_conf) / self.resolution))
        return not any([self.is_in_collision(np.array(conf)) for conf in np.linspace(prev_conf, current_conf, number_of_configurations_to_check, endpoint=True)])

    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1 - conf2, 2)) ** 0.5


class Building_Blocks_UR5e(Building_Blocks):
    def is_in_collision(self, conf) -> bool:
        # Implement UR5e-specific collision detection logic
        return super().is_in_collision(conf)


class Building_Blocks_UR3e(Building_Blocks):
    def sample(self, goal_conf) -> np.array:
        if random.random() < self.p_bias:
            return np.array(goal_conf)
        else:
            joint_4_direction = 1 # 1 for positive, -1 for negative, has to be constant for the path
            config = (np.random.uniform(-np.pi, np.pi), np.random.uniform(-np.pi, np.pi), np.random.uniform(-np.pi, np.pi))
            conf = config_autocomplete(config, joint_4_direction)
            return np.array(conf)
    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration
        """
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3], s2[0:3], self.ur_params.sphere_radius[self.ur_params.ur_links[i]],
                        self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        # arm - obstacle collision <Currently we don't have obstacles in our environment for simpliicity>
        # for joint, spheres in global_sphere_coords.items():
        #     for sphere in spheres:
        #         for obstacle in self.env.obstacles:
        #             if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
        #                 return True

        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            if joint == self.ur_params.ur_links[0]:
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        # y - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[1] + self.ur_params.sphere_radius[joint] > 0.5: # could be slightly buggy
                    return True

        # x - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4: # could be slightly buggy
                    return True

        # plate - arm collision


        # wrist3_spheres = global_sphere_coords[self.ur_params.ur_links[5]]
        # plate_center = np.array([0.0, 0.0, 0.0])  # center of the plate
        # plate_radius = 0.297  # radius of the plate (A4 size)

        # # Add spheres around the plate
        # plate_spheres = []
        # for theta in np.linspace(0, 2 * np.pi, num=20):
        #     x = plate_center[0] + plate_radius * np.cos(theta)
        #     y = plate_center[1] + plate_radius * np.sin(theta)
        #     z = plate_center[2]
        #     plate_spheres.append([x, y, z, 0.0])  # 0.0 is the radius of the sphere

        # # Add the plate spheres to the wrist3_spheres list
        # wrist3_spheres.extend(plate_spheres)

        # for sphere in wrist3_spheres:
        #     if sphere_collision(sphere[0:3], plate_center, self.ur_params.sphere_radius[joint], plate_radius):
        #         return True

        # # plate - floor collision
        # if plate_center[2] - plate_radius < 0.0:
        #     return True
        return False

