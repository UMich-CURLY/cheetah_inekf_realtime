# cheetah_inekf_realtime
This project contains a contact-aided invariant-ekf for MiniCheetah. It directly takes the contact estimation results from our [deep-contact-estimator](https://github.com/UMich-CURLY/deep-contact-estimator) via LCM, and output the estimated pose in ROS.

This repository has the following features:
* Directly compatible with the [deep-contact-estimator](https://github.com/UMich-CURLY/deep-contact-estimator).
* Use ROS for easier communication with other robotics program.
* Tested real-time performance on a Jetson AGX Xavier along with the deep contact estimator.

## Dependencies
* [lcm 1.4.0](https://github.com/lcm-proj/lcm/releases/tag/v1.4.0)
* [invariant-ekf/devel](https://github.com/RossHartley/invariant-ekf/tree/devel) ***Note***: Remember to checkout to the `devel` branch.
* Eigen3
* Boost
* YAML

## Configuration
### Parameters can be modified in `config/settings.yaml`:
* `project_root_dir`: Filepath to your installation directory for this repo
* `estimator_enable_debug`: Enable debug print on the screen.
* `estimator_publish_visualization_markers`: Enable LCM publisher for the estimated pose.
* `estimator_lcm_pose_channel`: Name of the LCM channel for output robot pose.
* `estimator_static_bias_initialization`: Enable static bias initialization using the first several measurements from IMU.
* `system_enable_pose_publisher`: Enable pose logger and publish the robot pose over ROS. Enable this will write down the estimated pose in a txt file and publish the pose to ROS at the same time.
* `system_inekf_pose_filename`: Path for the logged txt file. Kitti means the file will be recorded following the [Kitti format](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). 
*  `system_inekf_tum_pose_filename`: Path for the logged txt file in [TUM format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats). 

## Helpful Commands:

### Generating LCM Types:
1. `cd cheetah_inekf_lcm_root_directory/scripts`
2. `bash ./make_types.sh`
    
### Running Cheetah Estimator
1. cd `~/${PATH_TO}/catkin_ws`
2. In a new terminal in the catkin_ws, do catkin_make (perhaps multiple times)
3. Run `source ~/devel/setup.bash`
4. `roslaunch cheetah_inekf_lcm cheetah_estimator`
5. Run `lcm-logplayer-gui NAME_OF_LCM_LOG_FILE_HERE`
6. The terminal should begin printing out the robot state if the settings.yaml output variables are enabled

### Visualizing InEKF in Rviz
1. Start running the cheetah estimator using the instructions above
2. Enter `rviz` in the terminal
2. Select `Add by topic` setting and select path
3. Changed fixed frame to the same value as `map_frame_id` in config/settings.yaml
4. The robot pose should begin being drawn in rviz


## Citation
If you find this work useful, please kindly cite our publication in 2021 Conference on Robot Learning:

* Tzu-Yuan Lin, Ray Zhang, Justin Yu, and Maani Ghaffari. "Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events." In Conference on robot learning. PMLR, 2021
```
@inproceedings{
   lin2021legged,
   title={Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events},
   author={Tzu-Yuan Lin and Ray Zhang and Justin Yu and Maani Ghaffari},
   booktitle={5th Annual Conference on Robot Learning },
   year={2021},
   url={https://openreview.net/forum?id=yt3tDB67lc5}
}
```
