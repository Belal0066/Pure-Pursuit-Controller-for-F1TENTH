# Pure Pursuit Controller for F1TENTH Simulation

## Overview

This package implements a **Pure Pursuit** path tracking controller for the F1TENTH autonomous racing simulator environment.  
Pure Pursuit is a geometrically based lateral control algorithm that computes steering commands to follow a reference path defined by waypoints.  
The controller subscribes to odometry and publishes drive commands (`AckermannDriveStamped`) for the simulated racecar.

[Screencast from 2025-09-18 05-01-14.webm](https://github.com/user-attachments/assets/2448e7f0-a333-4d62-8cbb-e63f5aaa8ccd)

 
***

## How It Works

The controller receives:

- The car's current pose and velocity from the `/ego_racecar/odom` topic
- A reference path represented as a CSV file of waypoints loaded at startup

### Core Algorithm

1. **Find the closest waypoint** on the path to the car's current position.
2. **Select a “lookahead” waypoint** at a fixed distance ahead on the path.
3. **Transform the lookahead point** into the vehicle's coordinate frame.
4. **Calculate curvature and steering angle** using the Pure Pursuit formula:  
   $$\delta = \arctan\left(\frac{2L \sin(\alpha)}{l_d}\right)$$  
   where $$L$$ is the wheelbase, $$\alpha$$ is the angle to the lookahead point, and $$l_d$$ is the lookahead distance.
5. **Publish steering and speed commands** to drive the car along the reference trajectory.

The controller continuously loops at ~50Hz ensuring accurate and smooth path following.

***

## Directory Structure

Assuming this structure in your workspace at `/sim_ws/src`:

```
pure_pursuit_working/
├── config/
│   └── pure_pursuit.yaml                # ROS 2 node parameters
├── launch/
│   └── pure_pursuit.launch.py           # Launch file loading params and node
├── pure_pursuit_working/
│   ├── __init__.py                      # Required Python package file
│   └── pure_pursuit_node.py             # Main controller node code
├── package.xml
├── setup.py
└── setup.cfg
```

***

## Docker Image

A prebuilt Docker image with the Pure Pursuit package integrated is available on Docker Hub:

[f1tenth_gym_ros_with_pure_pursuit](https://hub.docker.com/repository/docker/belal0066/f1tenth_gym_ros_with_pure_pursuit/general)

Use this image to quickly run the F1TENTH simulator and controller:

```bash
docker pull belal0066/f1tenth_gym_ros_with_pure_pursuit:latest

docker run -it --rm belal0066/f1tenth_gym_ros_with_pure_pursuit:latest
```

For development, mount your workspace into the container or rebuild the image as needed.

***

## Running the Controller on the Simulator

### Step 1: Start the Simulator

Make sure your simulator is running with the **Austin** map loaded. If you replaced the Levine map files as per instructions, launch normally:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

# Create the package structure (if not created already)
ros2 pkg create --build-type ament_python pure_pursuit_working --dependencies rclpy geometry_msgs nav_msgs ackermann_msgs tf2_ros tf_transformations

# Install tf_transformations in ROS2 Foxy on Ubuntu 20.04
sudo apt-get update
sudo apt-get install ros-foxy-tf-transformations

# Build or rebuild the workspace
cd /sim_ws
colcon build --packages-select pure_pursuit_working
source install/setup.bash

# Launch the simulation environment
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Step 2: Run the Pure Pursuit Controller in another terminal

Run the controller node with parameters pointing to the Austin centerline waypoints:

```bash
ros2 run pure_pursuit_working pure_pursuit_node \
  --ros-args \
  -p csv_path:=/sim_ws/src/f1tenth_racetracks/Austin/Austin_centerline.csv \
  -p lookahead_distance:=1.5 \
  -p speed:=2.0 \
  -p wheelbase:=0.3302 \
  -p max_steering_angle:=0.4189
```

***

## Configuration Parameters

- `csv_path`: Path to CSV waypoint file describing the centerline of the track.
- `lookahead_distance`: Distance in meters ahead of the car to target for steering calculations.
- `speed`: Target forward speed in m/s.
- `wheelbase`: Vehicle wheelbase in meters.
- `max_steering_angle`: Steering angle limit in radians.

Adjust parameters for fine-tuning vehicle handling.

***

## Visualization

- Launch RViz configured to show:
  - The simulated map (`/map` topic)
  - The car’s pose and planned path/waypoints as markers or point clouds
- Verify waypoints overlap the map centerline accurately.

Yes, adding a link to your Docker Hub repository is a very good idea, especially in your README and documentation. It lets users easily find, pull, and run the pre-built Docker image with your pure pursuit package included, saving them from building the image themselves.

***



