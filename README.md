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
