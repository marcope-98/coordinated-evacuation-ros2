# robot-planning-ros2

## devtool.sh
In order to get access to the development tools and shortcuts to build, run and generate maps:

```console
$ cd robot-planning-ros2
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
$ cd robot-planning-ros2
$ rpl all
```
Wait until the simulator is setup: you'll notice that the logger stops posting messages in the terminal

2. In the second terminal execute command

```console
$ cd robot-planning-ros2
$ source install/setup.sh
$ ros2 launch rpl_ros2 rpl_ros2.launch.py n:=$number_of_robots$
```
Important: insert the number of shelfinos in the simulation instead of `$number_of_robots$`



## TODO:

- [x] Custom Messages
- [x] Adjust rpl/common.hpp to match shelfino geometric parameters
- [ ] Convert rpl_ros2 package into a composition
- [ ] Add rviz to simulator launch.py
