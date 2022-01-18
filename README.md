# temoto_visualization_manager

Visualization Manager uses RViz as the main platform for visualization, where each displayable data type
is displayed via dynamically loadable RViz plugins. Thus data can be visualized programmatically on demand.

## Installation

``` bash
cd catkin_ws/src
git clone --recursive https://github.com/temoto-framework/temoto_visualization_manager
cd ..
catkin build
```

## Examples
The following launch file will launch:
* **Process Manager**, which is used to launch RViz by the Visualization Manager
* **Visualization Manager**
* **Test node**, which loads couple of visualization plugins ([source code here](https://github.com/temoto-framework/temoto_visualization_manager/blob/main/temoto_visualization_manager/src/examples/visualization_manager_test.cpp))

``` bash
roslaunch temoto_visualization_manager visualization_manager_test.launch
```