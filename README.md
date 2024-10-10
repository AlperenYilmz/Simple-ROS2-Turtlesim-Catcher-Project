- This is a simple Turtlesim project in ROS2 Humble.
- Main turtle catches other spawned turtles (controlled by a parameter, which can be disabled)
- Uses custom interfaces
- Place the packages under your workspace, then:
```console
colcon build --packages-select custom_interfaces final_turtel_project
```
- Then run turtlesim_node, and then:
```console
ros2 run final_turtel_project spawner_node
ros2 run final_turtel_project main_controller
```
