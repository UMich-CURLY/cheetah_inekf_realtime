# cheetah_inekf_lcm
This project is the wrapper of invariant-ekf for MiniCheetah. It takes input from LCM/UDP, and performs EKF update on IMU-contact odometry. 

## Configuring yaml settings files
1. Change `enable_debug_output` param in config/settings.yaml to false if no debug output is wanted
2. Change `project_root_dir` param in config/settings.yaml to filepath to your installation directory for this repo

## Helpful Commands:

### Generating lcm types:
1. cd cheetah_inekf_lcm_root_directory/scripts
2. bash ./make_types.sh
    
### Running cheetah estimator
1. cd ~/pathto/catkin_ws
1. cd ~/pathto/catkin_ws
2. In a new terminal in the catkin_ws, do catkin_make (perhaps multiple times)
3. roslaunch cheetah_inekf_lcm cheetah_estimator.launch
4. source ~/devel/setup.bash
5. rosrun cheetah_inekf_lcm cheetah_estimator
