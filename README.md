# Implementation Plan

Team members: Kir Nagaitsev and Alec Blagg

### initialize particle cloud

We will randomly generate (x, y) coordinates within the width and height bounds of the map, along with a random starting angle, and set that as the pose for a particle. We will only keep this particle if it falls within the building, and will repeat this process until we have the desired particle count. Testing this will just require seeing the results of the initialization in RViz.

### update particles with motion model

We can calculate the distance traveled and angle change based on the previous odometry measurement, then apply that same change to each particle. We can test this by having only a few particles in RViz and seeing that they move in the same way that the robot moves. Noise will also be incorporated here, as discussed later.

### update particle weights with measurement model

Based on the pose of each particle, we will calculate its distance from the wall in front of it along with the walls to the left and right of it. We will subtract these from the actual readings and calculate 1 over the some of the absolute value of each difference, as done in class. If this method of weights doesn't give enough accuracy, we can calculate wall distances for the particles with more angles, since the robot LiDAR scan can offer 360 angles. We can test this after we add resampling, at which point it should be evident if we are approximating particle scans and weighing correctly.

### normalize particles and resample particles

Normalizing the particles will require making the weights of all the particles sum to 1, as we did in class. Then, we will randomly select values in the range [0, 1) for the number of particles we had, and place them in bins based on the normalized particle weights to decide the particles present in the next iteration, where particles with larger weights should have larger bins and thus will be more likely to have more particles in the next iteration. At this point, testing will be a matter of seeing if particles converge towards clouds around the accurate position.

### update estimated robot pose

### incorporate noise

We will incorporate noise in the distance traveled and angle change of each particle using Gaussian noise. We will ensure that this noise only has a small impact on the resulting value, as even small changes should help converge towards a better answer.

### timeline
