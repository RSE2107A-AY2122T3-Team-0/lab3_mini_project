# lab3_mini_project

## About
This package contains the limo_status_translator and limo_status_client nodes for lab3 mini project. This package depends on limo_ros, so make sure to have limo_ros within the same catkin workspace.

## Building and Running

1. git clone this repo and limo_ros into your catkin workspace
2. In your catkin workspace run
    ```bash
    catkin_make
    ```
3. Run limo_base, translator and client nodes using rosrun or use the launch file within this package
    ```bash
    roslaunch lab3_mini_project service_client.launch 
    ```