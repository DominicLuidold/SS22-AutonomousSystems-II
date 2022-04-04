# Abgabe dlu9576 - *ROS-02-Basic Commands and Topics*

Alle nachfolgenden Aufgaben setzen voraus, dass
```sh
roscore
```
ausgeführt wurde und eine entsprechende ROS-Instanz läuft.

### 1. Alle Turtles lassen sich mittels eines `turtle_teleop_key` Nodes per Tastatur steuern. In  diesem Fall haben alle Turtles dasselbe Verhalten.
```sh
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=turtlesim_2
rosrun turtlesim turtle_teleop_key
```

### 2. Jeder Turtle lässt sich getrennt über eine eigene Instanz von `turtle_teleop_key` steuern.
```sh
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=turtlesim_2 /turtle1/cmd_vel:=/turtle2/cmd_vel
rosrun turtlesim turtle_teleop_key
rosrun turtlesim turtle_teleop_key __name:=teleop_turtle_2 /turtle1/cmd_vel:=/turtle2/cmd_vel
```

### 3. Zwei oder mehrere Turtles werden über den `mimic` Node des Packages `turtlesim` miteinander verbunden. In diesem Fall sollte der zweite Turtle das Verhalten des ersten Turtle nachahmen. 
```sh
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=turtlesim_2 /turtle1/cmd_vel:=/turtle2/cmd_vel
rosrun turtlesim mimic /input/pose:=/turtle1/pose /output/cmd_vel:=/turtle2/cmd_vel
```

#### a) Eine Steuerung per Tastatur soll möglich sein
```sh
rosrun turtlesim turtle_teleop_key
```

#### b) Der Turtle soll mittels `rostopic pub` eine kontinuierliche Bewegung ausführen. Wie kann dabei der Nachrichtentyp bestimmt werden bzw. welche Nachrichten können dafür verwendet werden?
```sh
rostopic type /turtle1/cmd_vel
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

### 4. Erstelle eine weitere Instanz eines Turtles mittels des Spawn-Services (ein `turtlesim_node` ist hier ausreichend). Ermögliche eine getrennte Steuerung per `turtle_teleop_key`. 
```sh
rosrun turtlesim turtlesim_node
rosservice call /spawn 1 1 1 "turtle2"
rosrun turtlesim turtle_teleop_key
rosrun turtlesim turtle_teleop_key __name:=teleop_turtle_2 /turtle1/cmd_vel:=/turtle2/cmd_vel
```
