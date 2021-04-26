![Robot](https://github.com/Loonride/particle_filter_localization_project/raw/main/gifs/particle_filter_localization.gif)

# Implementation Plan

Team members: Kir Nagaitsev and Alec Blagg

### initialize particle cloud

We will randomly generate (x, y) coordinates within the width and height bounds of the map, along with a random starting angle, and set that as the pose for a particle. We will only keep this particle if it falls within the building, and will repeat this process until we have the desired particle count. Testing this will just require seeing the results of the initialization in RViz.

### update particles with motion model

We can calculate the distance traveled and angle change based on the previous odometry measurement, then apply that same change to each particle. We can test this by having only a few particles in RViz and seeing that they move in the same way that the robot moves. Noise will also be incorporated here, as discussed later.

### update particle weights with measurement model

Based on the pose of each particle, we will calculate its distance from the wall in front of it along with the walls to the left and right of it. We will subtract these from the actual readings and calculate 1 divided by the sum of the absolute value of each difference, as done in class. If this method of weights doesn't give enough accuracy, we can calculate wall distances for the particles with more angles, since the robot LiDAR scan can offer 360 angles. We can test this after we add resampling, at which point it should be evident if we are approximating particle scans and weighing correctly.

### normalize particles and resample particles

Normalizing the particles will require making the weights of all the particles sum to 1, as we did in class. Then, we will randomly select values in the range [0, 1) for the number of particles we had, and place them in bins based on the normalized particle weights to decide the particles present in the next iteration, where particles with larger weights should have larger bins and thus will be more likely to have more particles in the next iteration. At this point, testing will be a matter of seeing if particles converge towards clouds around the accurate position.

### update estimated robot pose

The estimated robot pose will attempt to keep track of the orientatioon and position of the robot. To do this we must keep track of the pose of all of the potential particles, which would involve updating their poses based on the movement instructions given, while also considering noise. This allows us to track the pose of the each of the particles, until eventually the location of the robot is found, at which point the estimated poses of the particles connected with this location can be used to know the pose of the robot.

### incorporate noise

We will incorporate noise in the distance traveled and angle change of each particle using Gaussian noise. We will ensure that this noise only has a small impact on the resulting value, as even small changes should help converge towards a better answer.

### timeline

When considering the timeline, we plan on breaking up the previous sections into approximately 3 separate general systems which we hope to complete by a certain date. The first of these will solely consist of the intialize_particle_cloud step, which we hope to complete by Thursday the 15th. Next will be the update_particles_with_motion_model and update_particle_weights_with_measurement_model, the two update steps, which we hope to complete by Friday the 16th. After that will be normalize_particles and resample_particles, with the goal being to complete this by Sunday the 18th. Lastly is the update_estimated_robot_pose, which would also be completed on the 18th.


# Writeup

## Objectives Description ##
The goal of this project was to be able to identify the location and pose of the robot within a map using algorithms learned in class. More specifically it was to start with a case where we know that the robot is within a certain area, in this case a house, and then to use sensor measumements, mainly odometry and laser scan, to over a series of movements pinpoint the robots position and orientation.

## High-Level Description ##
To solve this problem we used the ideas discussed in the "Robot Localization" as well as "Measurement Models for Range Finders" lectures. More specifically we used the idea of a likelihood field to identify the likelihood that our robot was in the position of any of 10000 randomly generated particles. This was combined with updating particle information based on movement, combined with noise, to offer repeated measurements and tests to allow for improved accuracy.

## Initialization of Particle Cloud #
# Code Location #
This code can be found in the ```initialize_particle_cloud``` function, and it uses other functions we wrote including ```in_building``` and ```normalize_particles```.

# Functions/Code Description #
*Initialize_Particle_Cloud*
This is the overarching function which performs the intialization. This involves a while loop over the number of particles expected in the cloud, within which the actual creation occurs. This includes the generation of a random x and y value within the largest bounds of the house, after which the ```in_building``` function is used to confirm that this is in an acceptable location. Afterwards final initialization of values, as well as the creation of the random yaw value, are performed. After this is simply the creation of the Pose() object followed by the creation of the new Particle() object and its addition to the particle cloud. Finally the particles are normalized and the cloud is published.

*In_Building*
The in building function was used to check a randomly generated x and y coordinate. This is due to the non-uniform shape of the house, which means that our randomly generated x and y, which was based on the extemes of the house, need to be checked to ensure they are properly placed. This check is done using the map object that represents the house as a series of integer values that represent whether a spot is valid.

*Normalize_Particles*
This function seeks to make the sum of the weights of the particles is equal to 1 for the purpose of resampling. This is done through two loops through the particle cloud. The first calculates the total weight of the particles, while the second modifies each particles weight to be its original weight divided by the total weight.


## Movement Model ##
# Code Location #
This is found in the ```update_particles_with_motion_model``` function.

# Functions/Code Description #
*Update_Particles_With_Motion_Model*
This function first calculates what movement the robot has made in the latest update by comparing its current odometry readings with the previous odometry readings. It then updates every particles pose to reflect the changes in position made using the previously mentioned calculation. This change also includes noise for every particle to allow to variation between a particle that was resampled multiple times to allow for more accurate localization.


## Measurement Model ##
# Code Location #
This is found in the ```update_particles_with_measurement_model``` function, which additionally uses the ```compute_prob_zero_centered_gaussian``` function.

# Functions/Code Description #
*Update_Particles_With_Measurement_Model*
This function uses the measurement model ideas presented in lecture when discussing likelihood fields for range finders. More specifically we consider the four cardinal directions and the measurements that the laser scan gives for each of these. Effectively what is being done here is that each particle is being moved to the location that is given for an obstacle for each of the cardinal directions, and the distance to the closest obstacle here is found. This means that particles that more accurately represent the robot will have smaller smaller dist values as this represents the particle being in a more accurate locations


## Resampling ##
# Code Location #
This is found in first the ```resample_particles``` function.

# Functions/Code Description #
*Resample_Particles*
First we create a list of "probabilities" which are just the weights of each particle. It then uses the ```np.random.choice``` function which selects items out of a list based on a list of probabilities, which are based on the weights. This therefore performs the work of randomly selecting which particles to keep. Lastly we set the pose of each particle in the particle cloud to be the pose of one of the newly selected particles.


## Incorporation of Noise ##
# Code Location #
The incorporation of noise can be found within the ```update_particles_with_motion_model``` function, more specifically when calculating move_distance, move_angle, and q.

# Functions/Code Description #
The noise was incorporated using the np.random.normal, which draws values from a normal distribution. In our case we did a normal distribution around 0 on a scale of 0.1, which means that we will get values that provide small changes to the move_angle, move_dist, and q values calculated for the particles allowing noise to be incorporated into the code.

## Updating Estimated Robot Pose ##
# Code Location #
This code is found in the ```update_estimated_robot_pose``` function.

# Functions/Code Description #
*Update_Estimated_Robot_pose*
This code works by calculating the average position of every particle in the particle cloud, and then using this average as the new estimated robot pose. This is done by iterating over the particle cloud and keeping a sum of each particles x, y, and yaw, dividing these by the number of particles, and setting the estimate to this new pose.


## Optimization of Parameters ##
# Code Location #
This is not seen in our code, as this was largely a manual process in which values were tested to see how the code performed, with comparisons made next to alternate runs with varying values

## Challenges ##
One challenge we faced was in the implementation of the updating the robot's estimated pose. This challenge was specifically caused by difficulty in properly having the newly calculated estimated pose be published to a ros topic.

Another challenge was in the running/efficiency of our code. One approach we took to dealing with this was only looking at the laser scan for 0, 90, 180, and 270 degrees so that less calculation was needed. In addition for some earlier testing the number of particles was reduced to allow smoother running. Our solution to this mainly involved identifying problem elements of the code we had written, such as unnecessary loops or code that was written that could be replaced by a library function that is better optimized, such as our approach to resampling particles.

## Future Work ##
We would like to take more time working on the noise element to improve the accuracy of our localization. Another element would be improving the efficiency of our program, as we had a couple of instances during our testing where we ran into problems with running the program with all desired particles, as discussed above in the Challenges section. As such working to improve the efficiency of our program would benefit future testing and use of localization, especially as it might apply to future work with the robot.

## Takeaways ##
* The use of discussion/comment writing to ensure that code written by one user is readable/understood by another so that the code can either be modified or utilized by the other member. A lot of prior work had been on individual projects in which comments are useful, but largely existed for the sake of ones own understanding of the code. Having to work in a way where the goal is to allow another user to read and work with your code helped drive comments to be more useful and utilized.
* The use of alternative methods of communicating/discussing the project when unable to meet in person. This includes things like learning how to effectively breakdown problems for discussion, as well as how to use tools such as zoom to allow communication and screen sharing. In addition things like working on how to brainstorm potential ideas and approaches that could be taken.
