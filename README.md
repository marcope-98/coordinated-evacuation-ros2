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