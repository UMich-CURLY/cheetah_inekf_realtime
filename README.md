# cheetah_inekf_lcm
This project is the wrapper of invariant-ekf for MiniCheetah. It takes input from LCM/UDP, and performs EKF update on IMU-contact odometry. 

## Configuring yaml settings files
1. Change `lcm_enable_debug_output` param in config/settings.yaml to false if no debug output is wanted for receiving lcm messages
2. Change `project_root_dir` param in config/settings.yaml to filepath to your installation directory for this repo
3. Change `estimator_enable_debug` param in config/settings.yaml to true if you want to view the state of the inekf in the terminal while it is running
4. Change `estimator_publish_visualization_markers` param in config/settings.yaml to true if you want to publish the robot pose over LCM channel "LCM_POSE_CHANNEL"
5. Change `estimator_lcm_pose_channel` param in config/settings.yaml to true if you want to change the name of the LCM channel that the robot pose is published over
6. Change `estimator_static_bias_initialization` param in config/settings.yaml to true if you want to initialize static bias for the IMU 
7. Change `system_enable_pose_publisher` param in config/settings.yaml to true if you want to save the robot pose to file and publish the robot pose over ROS
8. Change `system_inekf_pose_filename` and `system_inekf_tum_pose_filename` params in config/settings.yaml to a different filepath to specify which files you would like the robot poses to be saved to (the second is a tum syntax)

## Helpful Commands:

### Generating LCM Types:
1. cd cheetah_inekf_lcm_root_directory/scripts
2. bash ./make_types.sh
    
### Running Cheetah Estimator
1. cd ~/pathto/catkin_ws
2. In a new terminal in the catkin_ws, do catkin_make (perhaps multiple times)
3. Run `source ~/devel/setup.bash`
4. Open a new terminal and run `roscore`
5. In the previous terminal, run `rosrun cheetah_inekf_lcm cheetah_estimator`
6. Run `lcm-logplayer-gui NAME_OF_LCM_LOG_FILE_HERE`
7. The terminal should begin printing out the robot state if the settings.yaml output variables are enabled

### Debugging Inekf Output
1. Start running the cheetah estimator using the instructions above
2. Enter `rviz` in the terminal
2. Select `Add by topic` setting and select path
3. Changed fixed frame to the same value as `map_frame_id` in config/settings.yaml
4. The robot pose should begin being drawn in rviz
