# ROS2 Cheatsheet
There are a bunch of steps to working with ROS2 and a lot of links to navigate to find the steps (that all contain a bunch of extra info you don't really need to read every time), here's a handy doc to compile them!

## One Time Setup
Follow the steps on the [Environment Setup Page](https://comprobo23.github.io/How%20to/setup_your_environment) of the CompRobo website.

## QUICK ACCESS COMMANDS
### Build ROS2 workspace
>*You must do this any time you've made changes to your ROS2 packages*.

```
cd ~/ros2_ws
colcon build --symlink-install
```

I always cd into my base `/ros2_ws` directory to do this. You can build from each individual package's directory, but this will result in a bunch of `/build`, `/install`, and `/log` directories in each of your package directories. To build an individual package:

```
colcon build --symlink-install --packages-select my_package
```

### Source `setup.bash`
>*You must do this after you build your packages for your changes to apply.*
```
source ~/ros2_ws/install/setup.bash
```
The setup instructions have you add this line to your `.bashrc`, which runs every time you open a new terminal. You should do this, but if you make a change and rebuild and do not open a new terminal remember to source again!

## Getting Started (Python)
### Create a Package
> A summary of the [ROS2 Creating a Package instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

1. cd into your `src` directory
    ```
    cd ~/ros2_ws/src
    ```
2. Make the package
    ```
    ros2 pkg create --build-type ament_python <package_name>
    ```
    You can use optional flag `--node-name my_node` to create a blank node with this command.

### Make a New Node
> A summary of the [Day 2 activity](https://comprobo22.github.io/in-class/day02#coding-exercises)

Here is some barebones code for a node using `rclpy`, which is the Python package we will use to interface with ROS2.

```python
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
``` 

### Run Your Node
1. Add your package to `setup.py`
    ```python
        entry_points={
            'console_scripts': [
                'example_node = example_pkg.example_node:main',
            ],
        },
    ```
2. Build & source: [QUICK ACCESS COMMANDS](#quick-access-commands)
3. Run the node!
    ```
    ros2 run example_pkg example_node
    ```

### Interfaces
> A summary of [About ROS Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)

Interfaces are the methods through which ROS nodes communicate with each other. There are three types: messages, services, and actions. Messages will be the most frequently used in this course.

#### Messages
`.msg` files are included in the `/msg` directory of a package. They contain fields consisting of a type, name, and optionally a default value.

Documentation for message types included in your ROS2 Humble install are on the [ROS2 docs page](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html). You can also find the fields of the message with:
```
ros2 interfaces show pkg/msg/TYPE
```
You will most commonly interface with:
* [`std_msgs`](https://docs.ros2.org/latest/api/std_msgs/msg/): includes basic types like String, Bool, ints, floats, etc.
* [`geometry_msgs`](https://docs.ros2.org/latest/api/geometry_msgs/msg/): includes types to define movement and position in space.
* [`sensor_msgs`](https://docs.ros2.org/latest/api/sensor_msgs/msg/): includes types for sensors, particularly useful for camera information

You can import these included types with:
```python
from std_msgs.msg import [TYPE]
from geometry_msgs.msg import [TYPE]
from sensor_msgs.msg import [TYPE]
```

#### Services
TODO
#### Actions
TODO

### Publishing & Subscribing
> A summary of the [Day 2 activity](https://comprobo22.github.io/in-class/day02#coding-exercises)
#### Publisher
Create a publisher with:
```python
self.publisher = self.create_publisher(msg_type: Any, topic: str, qos_profile: QoSProfile | int)
```
The QoSProfile defines the Quality of Services policies for the topic. It's not important to go into detail here, but you can read more on the [Quality of Services settings page](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html). It's generally fine to put an `int` here which represents the depth field of the QoSProfile, which is the queue size that is buffered for the topic. Putting in 10 by default works well.

Once the publisher is created, you can publish messages of the correct type to it with:
```python
self.publisher.publish(msg)
```

#### Subscriber
Create a subscription with:
```python
self.subscription = self.create_subscription(msg_type: Any, topic: str, callback, qos_profile: QoSProfile | int)
```
The callback function determines what should occur when the node receives data from the topic. The callback function must have the message as an input parameter, but otherwise can do anything you'd like.

### Debugging Tools
#### Useful ROS2 Commands
* List all topics currently running
    ```
    ros2 topic list
    ```
* Print the output of a topic
    ```
    ros2 topic echo [TOPIC]
    ```
* Get the fields of a message type
    ```
    ros2 interfaces show pkg/msg/TYPE
    ```
#### rqt
TODO
#### rviz2
TODO
#### bag Files
TODO