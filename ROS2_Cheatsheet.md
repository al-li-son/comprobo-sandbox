- [ROS2 Humble Cheatsheet (Python)](#ros2-humble-cheatsheet-python)
  - [One Time Setup](#one-time-setup)
  - [QUICK ACCESS COMMANDS](#quick-access-commands)
    - [Build ROS2 workspace](#build-ros2-workspace)
    - [Source `setup.bash`](#source-setupbash)
  - [Getting Started](#getting-started)
    - [Create a Package](#create-a-package)
    - [Make a New Node](#make-a-new-node)
    - [Run Your Node](#run-your-node)
    - [Interfaces](#interfaces)
      - [Messages](#messages)
      - [Services](#services)
      - [Actions](#actions)
    - [Publishing \& Subscribing](#publishing--subscribing)
      - [Publisher](#publisher)
      - [Subscription](#subscription)
  - [Launch Files](#launch-files)
  - [Debugging Tools](#debugging-tools)
    - [Useful ROS2 Commands](#useful-ros2-commands)
    - [rqt](#rqt)
    - [rviz2](#rviz2)
    - [rosbag Files](#rosbag-files)
      - [Recording a Bag](#recording-a-bag)
      - [Playing a Bag](#playing-a-bag)

# ROS2 Humble Cheatsheet (Python)
There are a bunch of steps to working with ROS2 and a lot of links to navigate to find the steps (that all contain a bunch of extra info you don't really need to read every time), here's a handy doc to compile them!

[Mike Ferguson's ros2_cookbook](https://github.com/mikeferguson/ros2_cookbook) is a good reference for a lot of this information. This document includes the most commonly used parts for CompRobo.

## One Time Setup
Follow the steps on the [Environment Setup Page](https://comprobo23.github.io/How%20to/setup_your_environment) of the CompRobo website.

## QUICK ACCESS COMMANDS
### Build ROS2 workspace
>*You must do this any time you've made new ROS2 packages/nodes*.

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

## Getting Started
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

ROS will be able to find packages even if they are nested in the `/src` directory, so you can make multiple packages in one Git repo inside `/src`, for example.

### Make a New Node
> A summary of the [Day 2 activity](https://comprobo22.github.io/in-class/day02#coding-exercises)

Nodes live in a directory inside the package of the same name as the package.

Here is some barebones code for a node using `rclpy`, which is the Python package we will use to interface with ROS2. [`rclpy` documentation](https://docs.ros2.org/latest/api/rclpy/) is a good reference for using it.

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
`.msg` files are included in the `msg/` directory of a package. They contain fields consisting of a type, name, and optionally a default value.

Documentation for message types included in your ROS2 Humble install are on the [ROS2 docs page](https://docs.ros2.org/latest/api/). You can also find the fields of the message with:
```
ros2 interface show pkg/msg/TYPE
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
Services consist of a request from a client and a response from a server. The client (requester) sends information to
a server (responder) and waits for it to do a short computation and return a result.

`.srv` files are included in the `srv/` directory of a package. They contain a request and a response message type, separated by a `---`. 

Examples can be found on the [ROS2 Services docs page](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

ROS2 comes with a few standard services defined in the `std_srv` package. 

#### Actions
Actions are a long-running request/response communication. Unlike services, they can run for many seconds, provide feedback while they are occuring, and be inturrupted.

`.action` files are included in the `action/` directory of a package. They contain a request type, a response type, and a feedback type separated by `---`. 

Examples can be found on the [ROS2 Actions docs page](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

ROS2 comes with an example action, Fibonacci, found in the [`example_interfaces`](https://docs.ros2.org/latest/api/example_interfaces/action/) package.

### Publishing & Subscribing
> A summary of the [Day 2 activity](https://comprobo23.github.io/in-class/day02)

Note that a single node can have both publishers and subscribers. For simplicity, the example nodes are separately a publisher and a subscriber.

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

#### Subscription
Create a subscription with:
```python
self.subscription = self.create_subscription(msg_type: Any, topic: str, callback, qos_profile: QoSProfile | int)
```
The callback function determines what should occur when the node receives data from the topic. The callback function must have the message as an input parameter, but otherwise can do anything you'd like.

## Launch Files
Launch files are useful for when you want to run multiple nodes at once. It's usually best create a `launch/` directory in your package to put launch files. A basic launch file follows the following format:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_name', # required
            executable='exec_name', # required, name of python script that spins the node
            name='node_name', # optional, useful for renaming nodes if you want to run multiple of the same node
            parameters=[ # optional, if your node takes arguments
                {'param1': value},
                {'param2': value},
            ],
            remappings=[ # optional, can remap topic names to other names, useful for running multiple of the same node
                {'topic1', 'new_name1'},
                {'topic2', 'new_name2'},
            ]
        ),
        # Repeat with more nodes     
    ])
```

To run a launch file, you need to add it to your setup.py in the `data_files` list:
```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/LAUNCH_FILE_NAME.py'])
    ],
```
Then you **need to rebuild and resource** because the launch file will not be placed in the `/share` directory until the package is built.

Launch files are run with:
```
ros2 launch pkg launch.py
```

## Debugging Tools
### Useful ROS2 Commands
* List all nodes currently running
    ```
    ros2 node list
    ```
* List all topics currently running
    ```
    ros2 topic list
    ```
* Print the output of a topic
    ```
    ros2 topic echo [TOPIC]
    ```
* Get the type of a topic
    ```
    ros2 topic type [TOPIC]
    ```
    OR
    ```
    ros2 topic info [TOPIC]
    ```
* Get the fields of a message type
    ```
    ros2 interface show pkg/msg/TYPE
    ```
    Also works for services and actions, replace input with `pkg/srv/TYPE` or `pkg/action/TYPE`
### rqt
rqt is useful for visualizing the communication between your ROS nodes.

Launch rqt with:
```
rqt
```
To see the node graph, navigate to:
```
Plugins -> Introspection -> Node Graph
```
rqt will also be useful for viewing the stream from the camera with:
```
Plugins -> Visualization -> Image View
```
### rviz2
rviz2 is useful for visualizing the sensor and odometry data from your robot.

Launch rviz2 with:
```
rviz2
```
You can add topics to visualize with the `add` button on the bottom left. It's recommended to add `/device_pose`, which will show a vector representing the position and orientation of the robot. 

There are a lot of transforms under the `/tf` topic, `/base_link` is the robot coordinate frame.

`/scan` is also useful to visualize the lidar scan from the robot.

### rosbag Files
Bag files are recordings of ROS topic data. They can be very useful for debugging, iterating on your code without having to run a Neato, and demos.

#### Recording a Bag
```
ros2 bag record [TOPIC(s)] -o bag-file-name
```

It's best not to record all topics, especially for the Neato, as the file can get very large. Specifically, don't record /camera/image_raw or /gazebo/.

A good go-to command to record most of the information you'll need with a Neato:
```
ros2 bag record /accel /bump /odom /cmd_vel /scan /stable_scan /projected_stable_scan /tf /tf_static -o bag-file-name
```

#### Playing a Bag
```
ros2 bag play /path/to/bag
```
While a bag is playing you'll be able to view and echo topics and run nodes to interact with these topics exactly as if the Neato or simulator were running.
