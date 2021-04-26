#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random
import time

from likelihood_field import LikelihoodField


def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    return


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        self.likelihood_field = LikelihoodField()

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        time.sleep(2)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

        self.first_try = False


    def get_map(self, data):
        print(data.info)
        self.map = data
    
    def in_building(self, x, y):
        # helper function that confirms whether a given x, y are within the house
        grid_x = math.floor(x)
        grid_y = math.floor(y)
        grid = self.map.data
        index = grid_y * self.map.info.width + grid_x
        if index < len(grid) and grid[index] == 0:
            return True
        return False

    def initialize_particle_cloud(self):
        # print(self.likelihood_field.get_closest_obstacle_distance(10, 10))

        # this generates a bunch of particles to fill the particle cloud
        
        while len(self.particle_cloud) < self.num_particles:
            # randomly choose x and y based on height and width of map and confirm they are in the house
            map_x = random_sample() * self.map.info.width
            map_y = random_sample() * self.map.info.height
            if not self.in_building(map_x, map_y):
                continue
            
            map_x = map_x - self.map.info.width * 0.5 - 10
            map_y = map_y - self.map.info.height * 0.5 - 10
            x = map_x * self.map.info.resolution
            y = map_y * self.map.info.resolution
            z = random_sample() * 360
            z = np.deg2rad(z)
            # finalize x, y, and z(yaw) value

            # create new pose
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0
            q = quaternion_from_euler(0.0, 0.0, z)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            new_particle = Particle(p, 1.0)

            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # calculate total weight
        total_weight = 0
        for particle in self.particle_cloud:
            total_weight = total_weight + particle.w

        # recalculate particles weight
        new_tot = 0
        for particle in self.particle_cloud:
            particle.w = particle.w / total_weight
            new_tot += particle.w

        # diff = 1 - new_tot
        # if diff < 0:
        #     for particle in self.particle_cloud:
        #         if particle.w > abs(diff):
        #             particle.w += diff
        #             return
        #     return
        # self.particle_cloud[0].w += diff



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # new_cloud = []
        # create new probs list which is the weight of each particle
        probs = list(map(lambda p: p.w, self.particle_cloud))

        # new particle cloud calculated with the random.choice function
        new_cloud = np.random.choice(self.particle_cloud, size=self.num_particles, p=probs)

        # total_weight = 0
        # for particle in self.particle_cloud:
        #     total_weight = total_weight + particle.w

        # for i in range(self.num_particles):
        #     rand_num = random_sample()
        #     cur_sum = 0
        #     for particle in self.particle_cloud:
        #         cur_sum = cur_sum + particle.w
        #         if rand_num < cur_sum:
        #             new_cloud.append(particle)
        #             break
        
        # moves new particles from new_cloud to the particle cloud
        for i in range(self.num_particles):
            p = self.particle_cloud[i]
            new_p = new_cloud[i]
            p.pose.position.x = new_p.pose.position.x
            p.pose.position.y = new_p.pose.position.y
            p.pose.orientation.x = new_p.pose.orientation.x
            p.pose.orientation.y = new_p.pose.orientation.y
            p.pose.orientation.z = new_p.pose.orientation.z
            p.pose.orientation.w = new_p.pose.orientation.w

        return


    def resample_particles_2(self):
        new_cloud = sorted(self.particle_cloud, key=lambda p: p.w)
        self.particle_cloud = new_cloud[(len(new_cloud) - 100):]


    def robot_scan_received(self, data):
        # wait until initialization is complete
        if not(self.initialized):
            return

        if self.first_try:
            # self.update_particles_with_motion_model()

            self.update_particle_weights_with_measurement_model(data)

            self.normalize_particles()

            self.resample_particles_2()

            self.publish_particle_cloud()

            self.update_particle_weights_with_measurement_model(data, log=False)

            self.first_try = False

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        cur_x = 0
        cur_y = 0
        cur_yaw = 0

        for particle in self.particle_cloud:
            yaw = get_yaw_from_pose(particle.pose)

            cur_x = cur_x + particle.pose.position.x
            cur_y = cur_y + particle.pose.position.y
            cur_yaw = cur_yaw + yaw

        cur_x = cur_x / self.num_particles
        cur_y = cur_y / self.num_particles
        cur_yaw = cur_yaw / self.num_particles

        self.robot_estimate.position.x = cur_x
        self.robot_estimate.position.y = cur_y
        self.robot_estimate.orientation = quaternion_from_euler(0, 0, cur_yaw)

        return

    
    def update_particle_weights_with_measurement_model(self, data, log=False):
        for p in self.particle_cloud:
            p.w = 1

        # four the four cardinal directions find scan distance and use to update
        for i in [0, 90, 180, 270]:
            scan_dist = data.ranges[i]
            if scan_dist == math.inf:
                continue
            robot_yaw = np.deg2rad(i)
            for p in self.particle_cloud:
                p_yaw = get_yaw_from_pose(p.pose)
                p_x = p.pose.position.x
                p_y = p.pose.position.y

                tot_yaw = p_yaw + robot_yaw
                s_x = p_x + scan_dist * math.cos(tot_yaw)
                s_y = p_y + scan_dist * math.sin(tot_yaw)
                dist = self.likelihood_field.get_closest_obstacle_distance(s_x, s_y)
                if not dist or math.isnan(dist):
                    dist = 0
                
                prob = compute_prob_zero_centered_gaussian(dist, 0.1)
                p.w = p.w * prob

        # expected_scan = self.likelihood_field.get_closest_obstacle_distance(self.odom_pose.pose.position.x, self.odom_pose.pose.position.y)
        # print(f'Min: {min_scan}')
        # print(f'Expected: {expected_scan}')
        # for p in self.particle_cloud:
        #     x = p.pose.position.x
        #     y = p.pose.position.y
        #     d = self.likelihood_field.get_closest_obstacle_distance(x, y)
        #     if not d:
        #         d = 3.5
        #     p.w = abs()


    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # calculate movement by comparing current odometry with previous value
        prev = self.odom_pose_last_motion_update.pose
        curr = self.odom_pose.pose
        diff_x = curr.position.x - prev.position.x
        diff_y = curr.position.y - prev.position.y

        # calculate angle
        diff_angle = 0 if diff_y > 0 else math.pi
        if diff_x != 0:
            diff_angle = math.atan(diff_y / diff_x)
            if diff_x < 0:
                diff_angle += math.pi

        # calculate distance and angle
        dist = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))
        angle1 = get_yaw_from_pose(prev)
        angle2 = get_yaw_from_pose(curr)
        angle_change = angle2 - angle1

        # loop through particles updating their position with noise
        for part in self.particle_cloud:
            move_dist = dist + np.random.normal(0, 0.2)
            p_angle = get_yaw_from_pose(part.pose)# + np.random.normal(0, 0.2)
            move_angle = p_angle + (diff_angle - angle1) + np.random.normal(0, 0.1)
            p = Pose()
            p.position.x = part.pose.position.x + move_dist * math.cos(move_angle)
            p.position.y = part.pose.position.y + move_dist * math.sin(move_angle)
            p.position.z = 0
            q = quaternion_from_euler(0.0, 0.0, p_angle + angle_change + np.random.normal(0, 0.2)) # + np.random.normal(0, 0.25)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            part.pose = p


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









