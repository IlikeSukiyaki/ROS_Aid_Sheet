# Creating a ROS Package: Example

This guide demonstrates how to create a new ROS package within a catkin workspace.

---

## **1. Commands**
```bash
cd ~/catkin_ws/src
catkin_create_pkg learning_topic roscpp rospy std_msgs geometry_msgs turtlesim
```

### **Explanation**:
- **`cd ~/catkin_ws/src`**:
  - Navigates to the `src` directory of the catkin workspace (`catkin_ws`).
  - The `src` folder is where all ROS packages in the workspace are stored.

- **`catkin_create_pkg learning_topic roscpp rospy std_msgs geometry_msgs turtlesim`**:
  - **`catkin_create_pkg`**: Command to create a new ROS package.
  - **`learning_topic`**: The name of the new ROS package being created.
  - **Dependencies**:
    - `roscpp`: C++ client library for ROS.
    - `rospy`: Python client library for ROS.
    - `std_msgs`: Standard ROS message types (e.g., strings, integers).
    - `geometry_msgs`: Message types for representing geometric information (e.g., vectors, poses).
    - `turtlesim`: A simulation package often used for learning ROS.

---

## **2. Result**

The command creates a directory structure for the package (`learning_topic`) inside `~/catkin_ws/src`. The newly created package contains the following files and folders:

### **Folders**:
- **`include`**:
  - Typically used for header files in C++.
- **`src`**:
  - Contains the source code for the package (e.g., `.cpp` or `.py` files).

### **Files**:
- **`CMakeLists.txt`**:
  - Build configuration file for the package.
  - Specifies how the package should be built and its dependencies.
- **`package.xml`**:
  - Metadata file describing the package, its dependencies, and its purpose.

---

## **Summary**

This process demonstrates how to:
1. Create a new ROS package (`learning_topic`) in the `src` directory of a catkin workspace.
2. Specify dependencies (`roscpp`, `rospy`, `std_msgs`, `geometry_msgs`, and `turtlesim`) for the package during creation.
3. View the automatically generated structure and files for the package.

This workflow is a standard part of developing ROS packages within a catkin workspace.

# Creating a ROS Publisher in C++

This guide explains how to create a ROS publisher node in C++ that publishes velocity commands to a topic.

---

## **Code Explanation**

### **Code**:
```cpp
/**
 * Example: Publish velocity commands to the `/turtle1/cmd_vel` topic using `geometry_msgs::Twist`.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_publisher");

    // Create a NodeHandle
    ros::NodeHandle n;

    // Create a publisher to publish messages to `/turtle1/cmd_vel`
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Set the loop frequency
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        // Create and populate a Twist message
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;  // Forward velocity (m/s)
        vel_msg.angular.z = 0.2; // Rotational velocity (rad/s)

        // Publish the message
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);

        // Sleep for the remaining loop time to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
```

---

## **Code Breakdown**

1. **Include Necessary Headers**:
   - `ros/ros.h`: Includes core ROS functionality.
   - `geometry_msgs/Twist.h`: Defines the `Twist` message type used for velocity commands.

2. **Initialize the ROS Node**:
   ```cpp
   ros::init(argc, argv, "velocity_publisher");
   ```
   - Registers the node with the ROS master and names it `velocity_publisher`.

3. **Create a Node Handle**:
   ```cpp
   ros::NodeHandle n;
   ```
   - Allows interaction with the ROS system (e.g., advertise topics, subscribe to topics).

4. **Create a Publisher**:
   ```cpp
   ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
   ```
   - Advertises the `/turtle1/cmd_vel` topic with message type `geometry_msgs::Twist`.
   - The queue size is set to 10.

5. **Set Loop Frequency**:
   ```cpp
   ros::Rate loop_rate(10);
   ```
   - Specifies a loop frequency of 10 Hz (10 iterations per second).

6. **Main Loop**:
   ```cpp
   while (ros::ok()) {
       geometry_msgs::Twist vel_msg;
       vel_msg.linear.x = 0.5;
       vel_msg.angular.z = 0.2;
   ```
   - Ensures the node continues running unless ROS shuts down.
   - Creates and initializes a `Twist` message with linear and angular velocities.

7. **Publish Messages**:
   ```cpp
   turtle_vel_pub.publish(vel_msg);
   ROS_INFO("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);
   ```
   - Publishes the message to `/turtle1/cmd_vel`.
   - Logs the command using `ROS_INFO`.

8. **Maintain Loop Frequency**:
   ```cpp
   loop_rate.sleep();
   ```
   - Ensures the loop runs at the specified frequency (10 Hz).

---

## **Workflow Explanation**

### Steps to Implement a Publisher:
1. **Initialize the ROS Node**:
   - Use `ros::init()` to register the node with the ROS master.

2. **Register Node Information**:
   - Advertise the topic (`/turtle1/cmd_vel`) and specify the message type (`geometry_msgs::Twist`).

3. **Create Message Data**:
   - Define and populate the `Twist` message with velocity commands.

4. **Publish Messages**:
   - Use `publish()` to send the message to the topic.

5. **Loop with Specified Frequency**:
   - Use `ros::Rate` and `loop_rate.sleep()` to publish at a consistent rate.

---

## **Purpose**
This publisher sends velocity commands to control a turtle in the **turtlesim** simulation, allowing the turtle to move in a circular trajectory.

### File Name
- Save the code as `velocity_publisher.cpp`. 

# Configuring `CMakeLists.txt` for a ROS Node

This guide explains how to configure the `CMakeLists.txt` file for compiling and linking a ROS node.

---

## **Key Lines in `CMakeLists.txt`**

### **1. Declare a C++ Executable**
```cmake
add_executable(velocity_publisher src/velocity_publisher.cpp)
```
- **`add_executable`**:
  - Defines the executable to generate and specifies the source file(s) to compile.
  - **`velocity_publisher`**: Name of the executable.
  - **`src/velocity_publisher.cpp`**: The C++ source file to compile.

### **2. Link Libraries**
```cmake
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
```
- **`target_link_libraries`**:
  - Links the executable to the necessary libraries.
  - **`velocity_publisher`**: Name of the executable to be linked.
  - **`${catkin_LIBRARIES}`**: ROS libraries provided by catkin to enable ROS functionality.

---

## **How to Configure `CMakeLists.txt`**

### **1. Set the Source Code and Executable File**
- Use `add_executable()` to:
  - Specify the name of the executable.
  - Define the source file(s) that need to be compiled.

### **2. Set the Libraries to Link**
- Use `target_link_libraries()` to:
  - Specify the executable to be linked.
  - Define the libraries required for the executable to function properly within ROS.

---

## **Purpose of Each Command**

### **1. `add_executable`**
- Defines a build target (the executable to create).
- Associates the target with specific source files.

### **2. `target_link_libraries`**
- Ensures the executable has access to necessary libraries, such as:
  - ROS communication with the master.
  - ROS message types like `geometry_msgs::Twist`.

---

## **Example Workflow**

When building a ROS node:
1. Use `add_executable()` to specify the source file(s) and the executable name.
2. Use `target_link_libraries()` to link the executable with ROS libraries.

For this example:
- **Executable**: `velocity_publisher`.
- **Source File**: `src/velocity_publisher.cpp`.
- **Libraries**: `${catkin_LIBRARIES}` to enable ROS functionality.

---

## **Summary**

The `CMakeLists.txt` file is configured to:
1. Define the source files compiled into executables.
2. Specify the libraries required for these executables to communicate with ROS and utilize its resources.

This ensures that the node is built correctly and functions seamlessly within the ROS ecosystem.


# Running the `velocity_publisher` with TurtleSim

This guide outlines how to compile, run, and test a ROS publisher node with the TurtleSim simulation.

---

## **Steps**

### **1. Navigate to the Catkin Workspace**
```bash
cd ~/catkin_ws
```
- Change to the root directory of your catkin workspace.

### **2. Build the Workspace**
```bash
catkin_make
```
- Compile source code and generate executables.

### **3. Source the Workspace**
```bash
source devel/setup.bash
```
- Set environment variables so ROS can recognize your workspace and nodes.

### **4. Start the ROS Master**
```bash
roscore
```
- Launch the ROS master to manage node communication.

### **5. Launch TurtleSim**
```bash
rosrun turtlesim turtlesim_node
```
- Start the TurtleSim simulation.

### **6. Run the Publisher Node**
```bash
rosrun learning_topic velocity_publisher
```
- Execute the `velocity_publisher` node to send velocity commands.

---

## **What Happens?**

1. The `velocity_publisher` node publishes velocity commands (`linear.x = 0.5 m/s`, `angular.z = 0.2 rad/s`) to `/turtle1/cmd_vel`.
2. The TurtleSim window displays the turtle moving in a circular trajectory.
3. The terminal logs the published velocity commands:
   ```plaintext
   [INFO] Publish turtle velocity command [0.50 m/s, 0.20 rad/s]
   ```

---

## **Summary**
This workflow demonstrates:
- Compiling a ROS node with `catkin_make`.
- Setting up the environment with `source`.
- Running a custom publisher node to control the TurtleSim simulation via ROS topics.


# ROS Publisher in Python

This guide explains how to implement a ROS publisher node in Python to control the TurtleSim simulation.

---

## **Python Code**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    # Initialize the ROS node
    rospy.init_node('velocity_publisher', anonymous=True)

    # Create a publisher for the /turtle1/cmd_vel topic
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Set the publishing rate to 10 Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create and populate a Twist message
        vel_msg = Twist()
        vel_msg.linear.x = 0.5  # Linear velocity (m/s)
        vel_msg.angular.z = 0.2  # Angular velocity (rad/s)

        # Publish the message
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```

---

## **Explanation**

### **1. Import Required Libraries**
- `rospy`: ROS client library for Python.
- `geometry_msgs.msg.Twist`: Message type for velocity commands.

### **2. Initialize ROS Node**
```python
rospy.init_node('velocity_publisher', anonymous=True)
```
- Initializes the node with the name `velocity_publisher`.

### **3. Create Publisher**
```python
turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
```
- Publishes messages of type `Twist` to the topic `/turtle1/cmd_vel`.
- `queue_size=10`: Limits the number of messages queued for processing.

### **4. Define Publishing Loop**
- **Set Rate**: Ensures messages are published at 10 Hz:
  ```python
  rate = rospy.Rate(10)
  ```
- **Populate and Publish Messages**:
  ```python
  vel_msg = Twist()
  vel_msg.linear.x = 0.5  # Forward velocity
  vel_msg.angular.z = 0.2  # Rotational velocity
  turtle_vel_pub.publish(vel_msg)
  ```
  - Sends velocity commands to the TurtleSim node.

### **5. Graceful Shutdown**
```python
except rospy.ROSInterruptException:
    pass
```
- Handles interruptions gracefully (e.g., pressing Ctrl+C).

---

## **What Happens?**
- The node publishes velocity commands (`linear.x = 0.5 m/s`, `angular.z = 0.2 rad/s`) to `/turtle1/cmd_vel`.
- The TurtleSim moves in a circular trajectory.
- Published messages are logged to the terminal.

---

## **Summary**
- This Python script demonstrates a basic ROS publisher node.
- It integrates with TurtleSim to control the turtle's motion using velocity commands.

