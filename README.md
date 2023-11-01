# Coordinated evacuation of a team of robots in ROS2

Coordinated evacuation of a team of robots in ROS2 and C++.

The code in this repository was submitted in fulfillment of the requirements for the course '145874 - Robot Planning and its application'.

## devtool.sh
In order to get access to the development tools and shortcuts to build, run and generate maps:

```console
$ cd coordinated-evacuation-ros2
$ source ./devtool.sh
$ rpl
Available commands

	 rpl clean  : removes install, log, build and files in utils
	 rpl build  : performs build packages and source environment
	 rpl mapgen : calls MapGen to generate custom map
	 rpl run    : run simulation
	 rpl all    : clean -> mapgen -> build -> run
```

## Execution

Open two terminals:

1. In the first terminal execute command

```console
$ cd coordinated-evacuation-ros2
$ rpl all
```
Wait until the simulator is setup: you'll notice that the logger stops posting messages in the terminal

2. In the second terminal execute command

```console
$ cd coordinated-evacuation-ros2
$ source install/setup.sh
$ ros2 launch rpl_ros2 rpl_ros2.launch.py n:=$number_of_robots$
```
Important: insert the number of shelfinos in the simulation instead of `$number_of_robots$`

## Pipeline

The overall pipeline for the completion of the task comprises the following step:
### Initial Condition
First the obstacles, borders and gate positions are read by the `world_descriptor` node generating the following representation.

![](https://github.com/marcope-98/coordinated-evacuation-ros2/blob/master/media/step0.png)

### Obstacle and border inflation
Then the `world_descriptor` node inflates the obstacles and borders via Minkowski sum/difference and publishes the generated structure to a topic.

![](https://github.com/marcope-98/coordinated-evacuation-ros2/blob/master/media/step1.png)

### Roadmap Construction
The `roadmap_publisher` node reads the inflated obstacle list and computes a visibility graph, as follows. The resulting roadmap is then forward to a `shelfino_planner` node.

![](https://github.com/marcope-98/coordinated-evacuation-ros2/blob/master/media/step2.png)

### Path planning
The `shelfino_planner` node, reads the roadmap, adds the starting position of the robot and computes the shortest path using Dijkstra algorithm.

Then it finds the combination of Dubins manoeuvres that accomplishes the shortest traversed distance from the starting point to the gate. If a collision happens between a Dubins segment/curve and an obstacle/border, the node repeats the process until all the nodes have been visited.

Once all the robots have found a solution, the `shelfino_delay` node computes the delay necessary in order to avoid collision between the trajectories and forward the computed waypoints to the `shelfino_path_executor` node. In this project the robot with a minimum traveled distance has the priority in case of collision.

![](https://github.com/marcope-98/coordinated-evacuation-ros2/blob/master/media/step3.png)

### Path execution
The `shelfino_path_executor` node loops over each waypoint and implements a Lyapunov-based velocity controller in order to reach the desired target with the computed orientation.

The result of the 3rd demo is as follows:

![](https://github.com/marcope-98/coordinated-evacuation-ros2/blob/master/media/demo3.gif)


## TODO:

- [x] Custom Messages
- [x] Adjust rpl/common.hpp to match shelfino geometric parameters
- [ ] Convert rpl_ros2 package into a composition
- [ ] Add rviz to simulator launch.py
