# Abgabe dlu9576 - *ROS-05-Turtlebot3*

## Implementation documentation
All of the following specifications assume that these commands have been executed:
```sh
roscore
export TURTLEBOT3_MODEL=burger
```

## Teilaufgabe 2 - Wallfollower
> Bewegen Sie den Roboter vorwärts, bis dieser auf eine Wand trifft. Anschließend bewegen Sie den Roboter der Wand entlang und folgen dieser. Testen Sie Ihre Implementierung mit unterschiedlichen Umgebungsmodellen.

### Nodes

#### Follow Wall
Allows detecting, approaching and then following a wall. The node subscribes to `sensor_msgs/LaserScan` messages (used to detect the wall) and publishes `geometry_msgs/Twist` messages to the topic `cmd_vel`.

The node is started with the following command (which includes the Gazebo GUI):
```sh
roslaunch turtlebot3_dlu9576 follow_wall.launch
```

## Teilaufgabe 3 - Turtlebot3 auf ein Ziel steuern
> In dieser Teilaufgabe soll der Turtlebot3 sich zu einem Ziel bewegen. Das Ziel ist dabei kein konkreter Punkt im Koordinatensystem, sondern wird durch die Sensorwerte (Lasersensor) bestimmt. Es soll dabei das Ziel angesteuert werden, welches durch die Sensorwerte als das dem Roboter am nächsten bestimmt wird.
> 
> Erstellen Sie zu diesem Zweck ein einfaches Umgebungsmodell in Gazebo und verwenden Sie dieses mit Turtlebot3. Im einfachsten Fall kann nur ein Objekt enthalten sein um das Verhalten zu demonstrieren.

### Nodes

#### Obstacle Finder
Allows detecting and then approaching the nearest object. The node subscribes to `sensor_msgs/LaserScan` messages (used to detect the nearest object) and publishes `geometry_msgs/Twist` messages to the topic `cmd_vel`.

The node is started with the following command (which includes the Gazebo GUI):
```sh
roslaunch turtlebot3_dlu9576 obstacle_finder.launch
```
