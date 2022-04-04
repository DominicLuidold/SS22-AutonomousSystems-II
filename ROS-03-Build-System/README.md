# Abgabe dlu9576 - *ROS-03-Build System*
> Das/Die Packages inklusive Source-Code. Zusätzlich eine kurze Dokumentation der Architektur, verwendeten Nachrichten, Topics und anderen relevanten Eigenschaften der Implementierung. Diese Dokumentation sollte auch eine Beschreibung der Interaktionsmöglichkeiten mit dem System beinhalten und zeigen wie die einzelnen Knoten gestartet werden können (inklusive verwendeter Parameter).

## Implementation documentation
All of the following specifications assume that these commands have been executed:
```sh
roscore
rosrun turtlesim turtlesim_node
```

### Nodes

#### Turtlesim Control
Allows controlling the turtlesim node via a config file located in `turtlebot3_dlu9576/config/turtlesim_control.yaml`. The node publishes `geometry_msgs/Twist` messages to the topic `cmd_vel` using the `twist.linear.x` & `twist.angular.z` values defined in the yaml file.

The control mechanism can be enabled/disabled using the `turtlesim_control_srv` service (see below for more details).

The node is started with the following command and the parameters defined below to adjust the behaviour:
```sh
roslaunch turtlebot3_dlu9576 turtlesim_control.launch
```

| Parameter        | Default             | Format | Required | Description                              |
|------------------|---------------------|--------|----------|------------------------------------------|
| `turtle_name`    | `turtle1`           | `str`  | No       | Turtle topic name (default = `/turtle1`) |
| `node_name`      | `turtlesim_control` | `str`  | No       | Node name                                |
| `console_output` | `true`              | `bool` | No       | Log to console                           |

#### Turtlesim Status Monitor
Allows to monitor the turtlesim node based on three different topics:
* `/cmd_vel` consisting of `geometry_msgs/Twist`
* `/color_sensor` consisting of `turtlesim/Color`
* `/pose` consisting of `turtlesim/Pose`

which are aggregated into a single, custom `turtlebot3_dlu9576/TurtleStatus` message. This message is then published to the topic `/status` every time one of the three observed topics receive a new message with an *updated* value.

If the `path_analyzer` parameter is set to `true`, the path of the turtle is additionally visualised using `python3-pygame` (defined in `package.xml` as `exec_depend` & installed via `rosdep install`).

The node is started with the following command and the parameters defined below to adjust the behaviour:
```sh
roslaunch turtlebot3_dlu9576 turtlesim_monitor.launch
```

| Parameter        | Default             | Format | Required | Description                              |
|------------------|---------------------|--------|----------|------------------------------------------|
| `turtle_name`    | `turtle1`           | `str`  | No       | Turtle topic name (default = `/turtle1`) |
| `node_name`      | `turtlesim_control` | `str`  | No       | Node name                                |
| `console_output` | `true`              | `bool` | No       | Log to console                           |
| `path_analyzer`  | `true`              | `bool` | No       | Enable pygame visualization              |

#### Turtlesim Logger
Logs data to a `.json` using the required `path` parameter to determine the location. Subscribes to the `turtlebot3_dlu9576/TurtleStatus` message and writes the data received by the `turtlesim_monitor` node at a default frequency of 10 Hz.

The frequency can be adjusted using the `turtlesim_lograte_srv` service (see below for more details).

The node is started with the following command and the parameters defined below to adjust the behaviour:
```sh
roslaunch turtlebot3_dlu9576 turtlesim_logger.launch path:=/home/dlu9576/log.json
```

| Parameter     | Default             | Format | Required | Description                              |
|---------------|---------------------|--------|----------|------------------------------------------|
| `path`        | /                   | `str`  | Yes      | Path to store `.json` log file           |
| `turtle_name` | `turtle1`           | `str`  | No       | Turtle topic name (default = `/turtle1`) |
| `node_name`   | `turtlesim_control` | `str`  | No       | Node name                                |

### Services

#### Turtlesim Log Rate
Allows controlling the rate/frequency at which the `turtlesim_logger` node queries/writes the data to the `.json` file. Uses the `turtlebot3_dlu9576/LogRate` service message consisting of:
* `rate` (`float`) when sending the request
* and `old_rate` (`float`) & `new_rate` (`float`) as a response

The frequency can be adjusted with the following command:
```sh
rosrun turtlebot3_dlu9576 turtlesim_lograte_srv [rate]
```

| Parameter | Format         | Required |
|-----------|----------------|----------|
| `rate`    | `int`, `float` | Yes      |

#### Turtlesim Control
Allows to enable/disable the `turtlesim_control` node, therefore stopping the turtle from moving. Uses the `turtlebot3_dlu9576/ControlStatus` service message consisting of:
* `enabled` (`bool`) when sending the request
* and `old_status` (`bool`) & `new_status` (`bool`) as a response

The movemenet can be enabled/disabled with the following command:
```sh
rosrun turtlebot3_dlu9576 turtlesim_control_srv [status]
```

| Parameter | Format | Required |
|-----------|--------|----------|
| `status`  | `bool` | Yes      |