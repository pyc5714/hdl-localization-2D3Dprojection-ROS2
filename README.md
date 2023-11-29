## 1. docker image
```
docker pull yechanpark5714/inference_ros2:latest
```

## 2. docker container
```
nvidia-docker run --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume= *******your ws**********(ex. /data/ROS2_hdl_localization:/home/inference_ros2) \ 
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=2gb \
           --name=inference_ros2 \ 
           --env="DISPLAY=$DISPLAY" \
           yechanpark5714/inference_ros2:latest
```

## 3. package tree
```
catkin_ws
    ├── src
    ├── ndt_omp
    ├── hdl_localization
    ├── hdl_global_localization
    ├── fast_gicp
    ├── Segmentation_code
    ├── visualization
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── src
            ├── local_visualization.cpp
            ├── global_visualization.cpp
```

## 4. result


## 5. manual
you need 5 terminal windows

**default setting :** 
```
source /opt/ros/foxy/setup.bash
source /home/ROS2_hdl_localization/install/setup.bash
```
#### (1) run local_visulization 
```
ros2 run visualization local_visualization
```
#### (2) Segmentation
```
cd /home/ROS2_hdl_localization/src/Segmentation_code
python3 RunSegNode.py 
```
#### (3) launch hdl localization
```
ros2 launch hdl_localization hdl_localization_2.launch.py
```
#### (4) rviz
```
cd /home/ROS2_hdl_localization/src/visualization
rviz2 -d visualization.rviz
```
#### (5) bag play
```
cd /home/ROS2_hdl_localization
ros2 bag play 2022-09-19-10-00-32
```

![image](https://github.com/pyc5714/hdl-localization-2D3Dprojection-ROS2/assets/79192580/53f724fa-a18f-4a30-a2f2-fb49b614b43a)
