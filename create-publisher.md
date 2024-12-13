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

