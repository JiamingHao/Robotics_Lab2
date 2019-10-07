# COMS W4733 Robotics Lab 2

Usage:
------
In two terminals, each run one of the following two code snippets:
```shell
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/PATH/TO/bug2_0.world
```
```shell
$ cd [DIRECTORY CONTAINING bug2.py]
$ chmod +x bug2.py
$ ./bug2.py
```

Methods:
------
```python
def follow_boundary(self):
    """ Make the robot follow the boundary of the obstacle """
    ...
```
```python
def get_goal_direction(self):
    """ Make the robot rotate towards the goal point """
    ...
```
```python
def get_mline_distance(self):
    """ Calculate the distance from the robot's current position to the m-line """
    ...
```
```python
def get_mline_distance(self):
    """ Calculate the distance from the robot's current position to the m-line
    The distance is to the nearest point on the m-line
    """
    ...
```
```python
def get_distance_between(self, p1, p2):
    """ Calculate the distance between the two points, p1 and p2 """
    ...
```

Video:
------
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/iXrD4rcCp8I/0.jpg)](http://www.youtube.com/watch?v=iXrD4rcCp8I)
