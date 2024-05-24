# Track_Anything_Rapter(TAR)

## Description
Track Anything Raptor (TAR) is a ROS2-based aerial vehicle system that detects, segments and tracks objects using multimodal queries (text, images, clicks). TAR leverages pre-trained models (DINO, CLIP, SAM) for pose estimation and employs Visual Servoing for precise tracking, validated against Vicon-based ground truth on a PX4 Autopilot-enabled Voxl2 M500 drone.

## Dependencies
Note: The following are the dependencies currently used in the project. The project may work with other versions of the dependencies as well.
- Cuda 12.2
- Python 3.10
- Ubuntu 22.04
- ROS 2 Humble(Workstation) & Foxy(Drone)

## Installation
Step 1: Clone the repository
```bash
git clone https://github.com/tvpian/Project-TAR.git
``` 
Step 2: Install the required packages
```bash
pip install -r setup_files/project_tar_pip.txt
```
or (for Conda environment installation)
```bash
conda install --file setup_files/project_tar_v3.yaml
```
<!-- Step 2: Setup the FollowAnthing git repository as described in the [FollowAnything](https://github.com/alaamaalouf/FollowAnything) repository. -->

## Running the TAR system on a VOXL2 based drone using ROS2
https://github.com/tvpian/Project-TAR/assets/41953267/c45b2294-eaa9-4902-ac12-97a89a4d5038

This guide will walk you through the steps to set up and run the TAR system using a VOXL2 drone.

## Prerequisites

Ensure you have the following software and hardware:

- VOXL2 Drone
- Ground station with VLC player installed
- Python 3.10
- Necessary Python libraries

## Setup and Execution

Follow these steps in the specified order to start the TAR system:

### Step 1: Setup the Video Stream from VOXL2 Drone

Set up the video stream from your VOXL2 drone as described in the [VOXL2 RTSP Stream](https://docs.modalai.com/voxl-streamer/) documentation. VOXL2-based drones provide a video stream over RTSP out of the box.

To check if the RTSP stream is working, open the respective URL in a VLC player from your ground station.

### Step 2: Run the Vision Node

Once the RTSP stream is verified to be working, you can run the TAR system by executing the following commands on a terminal:

1. Change to the `FollowAnything` directory:
   ```bash
   cd FollowAnything
   ```

2. Run the Vision Node:
   ```bash
   python follow_anything_ros.py --desired_height 240 --desired_width 320 --path_to_video rtsp://192.168.8.1:8900/live --save_images_to outputs/ --detect box --redetect_by dino --tracker aot --queries_dir queries/apriltag_following/ --desired_feature 6 --plot_visualizations
   ```

The Vision Node is responsible for detecting and tracking the object of interest in the video stream and sending the object's location to the Controller Node.

### Step 3: Start the Controller Node

Open a new terminal and run the following command to start the Controller Node:
```bash
python test_ws/input_vel_fly_OG.py
```

The Controller Node is responsible for controlling the drone based on the object's location received from the Vision Node. The Controller Node sends velocity commands to the drone to follow the object of interest.

## Additional Notes

- Ensure all scripts and commands are executed in the proper environment with the necessary permissions.
- Verify the RTSP server URL and adjust it if necessary.
- Monitor each step for successful execution before proceeding to the next one.

By following these steps, you should be able to set up and run the TAR system successfully using your VOXL2 drone.

## Running the TAR system on Gazebo simulation using ROS2
https://github.com/tvpian/Project-TAR/assets/41953267/e6b06adf-a33e-4b7f-be39-77a906b7a03e

This guide will walk you through the steps to set up and run the Baylands Total-1 simulation environment.

## Additional Prerequisites for Gazebo Simulation

Ensure you have the following software installed on your system:

- Gazebo
- Microxe Agent
- QGroundControl(For making any manual changes to the drone's position)

## Setup and Execution

Follow these steps in the specified order to start the simulation:

1. **Start Gazebo**
    Note: You have to ensure PX4-Autopilot-related files are in the rapter_ws directory.
   ```bash
   cd rapter_ws/PX4-Autopilot
   make px4_sitl_default gazebo
   ```

2. **Start the Microxe Agent**
   ```bash
   # No specific command provided, ensure the Microxe agent is running
   ```

3. **Start QGroundControl**
   ```bash
   ./QGroundControl.AppImage
   ```

4. **Start the RTSP Server**
   ```bash
   python test_ws/rtsp_server.py
   ```

5. **Start the Camera Subscriber and RTSP Server Proxy**
   ```bash
   python test_ws/uav_camera_to_crutch_proxy.py
   ```

6. **Set the Desired Object in Motion**(Modify the set_sinusoidal_motion.sh file to set the desired object in motion)
   ```bash
   ./set_sinusoidal_motion.sh
   ```

7. **Start the Vision Node**
   ```bash
   cd FollowAnything/
   python follow_anything_ros.py --desired_height 240 --desired_width 320 --path_to_video rtsp://127.0.0.1:1234/video_stream --save_images_to outputs/ --detect box --redetect_by box --tracker aot --plot_visualizations
   ```

8. **Start the Offboard Controller**
   ```bash
   python test_ws/input_vel_fly.py
   ```

## Additional Notes

- Ensure all scripts and commands are executed in the proper environment with the necessary permissions.
- Verify the RTSP server URL and adjust it if necessary.
- Monitor each step for successful execution before proceeding to the next one.

By following these steps, you should be able to set up and run the Baylands Total-1 simulation environment successfully.

## Results
For the video demonstration of the project, please refer to the following [link](https://drive.google.com/drive/u/1/folders/1gO0R1qUqjNkcfFNRyeN-Uf0ZnvGIiMT8)
For the report of the project, please refer to the report available in the repository as [Project_Report.pdf](report.pdf)


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- [FollowAnything](https://github.com/alaamaalouf/FollowAnything) repository for the object tracking and following codebase.
