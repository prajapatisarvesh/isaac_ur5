# Nvidia Isaac Sim UR5

This repository contains implementation of UR5 in Nvidia Isaac Sim with ROS bridge and MoveIt. From [Universal Robot's GitHub](https://github.com/ros-industrial/universal_robot) the xacro was converted to urdf, and later imported as usd to Isaac sim. MoveIt configuration was written and using action graph and bridge the connectivity between UR5 in Isaac and ROS was established.
![UR5 Isaac](https://i.imgur.com/s46TTZU.png)
<hr/>

## Introduction

This repo is part of another project and this whole system does the work for me, but if you are looking for some more add-ons, fell free to fork and submit a PR. If you want to create a system with a different manipulator, Nvidia's documentation are really helpful and if get stuck, fell free to reach out to me.

## Ussage

```
# Clone the repo in your workspace and build it.
cd ~ && mkdir -p isaac_ur5_ws/src && cd ~/isaac_ur5_ws/src
git clone git@github.com:alpharomeo911/isaac_ur5.git
cd ..
catkin build
# Open Isaac Sim and open `urr5_ws.usd` in scene folders.
source devel/setup.bash
echo "source ~/isaac_ur5_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
# run the simulation in isaac sim by hitting the play button or by pressing space
# running the launch file 
roslaunch isaac_ur5 ur5_isaac_execution.launch
```

**Plan something in RViz and see the simulation in Isaac Sim**

![Working of the bridge](https://i.imgur.com/n23wSlU.gif)
