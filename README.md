
# Global Navigation

This repository introduces an innovative approach for outdoor navigation using ROS2.

## Packages

This repository comprises two packages:

- **local_navigation**: Reconstructs the navigated area into a grid map incorporating elevation and RGB data.
- **traversability_updater**: Computes a navigability score based on previously traversed areas.

The animation below highlights highly navigable areas in yellow and green. As the robot traverses the grass, all mapped zones with similar characteristics are marked as highly navigable.

![Navigation Demo](media/navigation_demo.gif)

## Usage

To reproduce our results, follow these instructions:

### Installation

- Requirements
    - Ubuntu 24.04
    - Ros2 Jazzy
    - Python 3.12.3
- Or use [distrobox](https://github.com/89luca89/distrobox)
    ```sh
    distrobox create --image ghcr.io/sloretz/ros:jazzy-desktop --name jazzy-desktop --home /path_to_container_home
    distrobox enter jazzy-desktop
    ```

- Install grid_map pkg
    ```sh
    sudo apt install ros-jazzy-grid-map-ros
    ```
- Clone [lidarslam](https://github.com/rsasaki0109/lidarslam_ros2) package to your workspace.
    ```sh
    cd workspace_folder/src/
    git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
    ```
- Clone the repository
    ```sh
    git clone https://github.com/midemig/global_navigation -b jazzy
    ```
- Install dependences and build workspace
    ```sh
    cd ..
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install 
    ```
- Install python dependences
    ```sh
    sudo apt install python3.12-venv
    python3 -m venv global_nav_env
    source global_nav_env/bin/activate
    pip3 install -r src/global_navigation/requirements.txt
    ```


### Launch Demo

1. Download and unzip the [demo bagfile](https://urjc-my.sharepoint.com/:u:/g/personal/juancarlos_serrano_urjc_es/EQI9T9RNYuFJg6reV-pq-7IBjMEeEo7RxaJCudMs9IyRTg?e=hSNyQB).
2. In a terminal, play the downloaded bag:
    ```sh
    source /opt/ros/jazzy/setup.bash
    cd bagfile_folder/
    ros2 bag play *.db3 --clock -p
    ```
3. In another terminal, execute:
    ```sh
    source global_nav_env/bin/activate
    export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
    ros2 launch local_navigation demo.launch.py
    ```




