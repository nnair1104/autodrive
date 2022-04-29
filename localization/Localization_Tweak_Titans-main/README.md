# Localization : Tweak Titans

## How to start the node:
First go to working directory then go to bash folder and execute `download_loc_bag.bash`. This downloads a rosbag that contains a prerecorded drive of a car. By using the prerecorded drive every team has the same driving-routes and the results are thus comparable.
However, you can also exchange running the bag with running the simulator (sim_base.launch).

Run `roslaunch freicar_localization evaluate.launch`. Overall this launch file will play the rosbag, start the street sign detector and starts the particle filter. When you ctrl/C after the rosbag has finished playing a mean localization error is displayed.

If you want to start the node in "live mode" while running the simulator with the sim_base.launch file you can run:
`rosrun freicar_localization freicar_localization_node _use_lane_regression:=false`

If you want to run sign detection while running the live node execute `rosrun freicar_sign_detect freicar_sign_detect_node`

If `use_lane_regression` (inside the evaluate.launch or while running the individual node) is set to true the filter subscribes to `/freicar_1/sim/camera/rgb/front/reg_bev` where it assumes it to be the birdseye lane regression map where every pixel is between 0 and 255. This map is then passed to the sensor_model class where you could implement your own sensor_model for the regression.  

The sampling used in the file `particle_filter.cpp` is LowVarianceSampling. If you want to use RouletteSampling, first comment line no. 484 and uncomment line no. 485. 

