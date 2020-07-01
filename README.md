# tmplanner
This repository contains a real-time capable informative motion planning framework for terrain monitoring applications using unmanned aerial vehicles (UAVs). The framework is suitable for monitoring either discrete or continuous target variables and in missions with online adaptivity requirements for targeted regions of interest.

The planner operates in a finite-horizon fashion, alternating between replanning and plan execution, while taking new sensor data into account. The replanning stage consists of two steps: (1) coarse 3D grid search in the UAV workspace and (2) optimization of this trajectory for maximized information/exploratory gain using an evolutionary scheme.

This README gives a short overview of the repository contents. For more information, please refer to the [wiki](https://github.com/ethz-asl/tmplanner/wiki).

**Author**: Marija Popović  
**Maintainer**: Marija Popović, m.popovic@imperial.ac.uk  
**Affiliation**: Smart Robotics Lab., Imperial College London (time of work: Autonomous Systems Lab., ETH Zurich)

<p align="center"><img src="https://preview.ibb.co/n7Q7OG/vlcsnap_2017_11_02_21h35m04s545.png" width="400" /></p>

***

## Bibliography

More information about our mapping and planning methods can be found in the following papers. Please cite them if you use our software in a scientific publication.

* For our discrete variable planner:

Marija Popović, Gregory Hitz, Juan Nieto, Inkyu Sa, Roland Siegwart, and Enric Galceran “**Online Informative Path Planning for Active Classification Using UAVs**”. In *IEEE Int. Conf. on Robotics and Automation* (ICRA), Singapore, May 2017.
```
@inproceedings{popovic2017icra,
  author={Popović, Marija and Hitz, Gregory and Nieto, Juan and Sa, Inkyu and Siegwart, Roland and Galceran, Enric},
  booktitle={Robotics and Autmation (ICRA), 2017 IEEE International Conference on},
  title={Online Informative Path Planning for Active Classification Using UAVs},
  year={2017},
  address={Singapore},
  month={May}
}
```

* For our continuous variable planner:

Marija Popović, Teresa Vidal-Calleja, Gregory Hitz, Inkyu Sa, Roland Siegwart, and Juan Nieto “**Multiresolution Mapping and Informative Path Planning for UAV-based Terrain Monitoring**”. In *IEEE/RSJ Int. Conf. on Intelligent Robots and Systems* (IROS), Vancouver, September 2017.
```
@inproceedings{popovic2017iros,
  author={Popović, Marija and Vidal-Calleja, Teresa and Hitz, Gregory and Sa, Inkyu and Siegwart, Roland and Nieto, Juan},
  booktitle={Intelligent Robots and Systems (IROS), 2017 IEEE International Conference on},
  title={Multiresolution Mapping and Informative Path Planning for UAV-based Terrain Monitoring},
  year={2017},
  address={Vancouver},
  month={September}
}
```


* For the integrated framework (including more extensive experimental results):

Marija Popović, Teresa Vidal-Calleja, Gregory Hitz, Inkyu Sa, Roland Siegwart, and Juan Nieto “**An informative path planning framework for UAV-based terrain monitoring**”. In *arXiv/1809.03870*, 2020.
```
@inproceedings{popovic2020,
  author={Popović, Marija and Vidal-Calleja, Teresa and Hitz, Gregory and Sa, Inkyu and Siegwart, Roland and Nieto, Juan},
  title={An informative path planning framework for UAV-based terrain monitoring},
  year={2020},
  url={http://arxiv.org/abs/1809.03870},
  eprint={1809.03870}
}
```

## Installation Instructions (Ubuntu)
To install this package with [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu):

1. Install additional system dependencies (swap kinetic for indigo as necessary):

```
sudo apt-get install python-wstool python-catkin-tools ros-kinetic-cmake-modules libyaml-cpp-dev
```

2. Set up a catkin workspace (if not already done):

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

3. Install the repository and its dependencies (with rosinstall):

```
cd src
wstool init
wstool set --git tmplanner git@github.com:ethz-asl/tmplanner.git -y
wstool update
wstool merge tmplanner/install/tmplanner.rosinstall
wstool update -j8
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. Use [catkin_build](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) to build the repository:

```
catkin build
```

## Code Structure

This repository contains three packages: two stand-alone planning frameworks corresponding to discrete (``tmplanner_discrete``) and continuous (``tmplanner_continuous``) variable monitoring, in addition to a toolset (``tmplanner_tools``) of convenience functions and simulation infrastructure used by both frameworks.

Each planning framework includes the following key components (same name under different package names):

* Node interface for ROS communication - ``tmplanner_node.cpp``
* Planning unit - ``tmplanner.cpp``
* Mapping unit - ``grid_map.cpp``

Simulation demos for both packages are provided, as described below.


## Simulation Demo Instructions
In this repository, we provide illustrative demos of both discrete (binary) and continuous target variable terrain monitoring in the Gazebo-based [RotorS](https://github.com/ethz-asl/rotors_simulator/wiki) environment. The simulation set-up includes a rotorcraft-type UAV equipped with a downward-facing camera, as per the algorithms' problem formulation.

The following steps outline the terminal commands to run an example for mapping a continuous variable. The same procedure can be applied to map discrete variables, by replacing for the ``tmplanner_discrete`` package name in the commands.

1. Start the simulation environment by running:

 ```
 $ roslaunch tmplanner_continuous monitoring_example.launch
 ```
 
 2. Start the planner node:
 
  ```
 $ roslaunch tmplanner_continuous tmplanner.launch
 ```
 
   > **Note**: The appropriate [parameters](https://github.com/ethz-asl/tmplanner/wiki/Parameters) must be loaded in the launch file.
 
 3. Initialize the planning routine via the rosservice:
 
  ```
 $ rosservice call /firefly/start_planning
 ```
You should now be able to see the UAV moving in the Gazebo window as the planning routine is executed.
 
## Acknowledgement

This work was funded by the European Community’s Horizon 2020 programme under grant agreement no 644227-Flourish and from the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract number 15.0029.  
http://flourish-project.eu

<p align="center"><img src="http://flourish-project.eu/fileadmin/bsdist/theme/img/flourish-logo-v5.svg" width="200" /></p>

## Contact

You can contact the maintainer for any question or remark:
* [Marija Popović](mailto:mpopovic@ethz.ch)
