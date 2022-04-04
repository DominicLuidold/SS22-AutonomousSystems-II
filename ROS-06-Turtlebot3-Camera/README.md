# Abgabe dlu9576 - *ROS-06-Turtlebot3 - Camera*

> Das/Die Packages inklusive Source-Code. Das Package soll es ermöglichen ein Kamerabild darzustellen (original und in Grautönen). Zusätzlich eine kurze Dokumentation der Implementierung. Diese Dokumentation sollte auch eine Beschreibung der Interaktionsmöglichkeiten mit dem System beinhalten und zeigen wie die einzelnen Knoten gestartet werden können (inklusive verwendeter Parameter).

## Dokumentation der Implementierung
Alle nachfolgenden Aufgaben setzen voraus, dass
```sh
roscore
```
ausgeführt wurde und eine entsprechende ROS-Instanz läuft.

### Ausführen des Packages `turtlebot3_camera_dlu9576`
Damit ein Kamerabild dargestellt wird, muss vorab ein Turtlebot3 in einem beliebigen Umgebungsmodell gestartet werden:
```sh
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Anschließend kann das Kamerabild mittels folgendem Befehl angezeigt werden (nach "Vorab-Installation" von `turtlebot3_camera_dlu9576.zip`):
```sh
roslaunch turtlebot3_camera_dlu9576 turtlesim_camera_vis.launch img_mode:=default
```

Der Parameter `img_mode` ist verpflichtend. Über diesen kann angegeben werden, ob das Bild mit Farben oder in einer Grauton-Variante angezeigt werden soll:
* `img_mode:=default` -> Bild mit Farben
* `img_mode:=gray` -> Bild in Grauton
