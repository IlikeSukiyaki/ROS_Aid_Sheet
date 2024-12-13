# ROS Subscriber in C++

This guide explains how to implement a ROS subscriber node in C++ that subscribes to a topic and processes incoming messages.

---

## **Code Overview**

### **1. Callback Function**
```cpp
void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}
```
- Triggered whenever a new message is received on the subscribed topic.
- Processes the `turtlesim::Pose` message and prints the `x` and `y` coordinates to the console.

### **2. Main Function**
```cpp
int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    ros::spin();

    return 0;
}
```
- **`ros::init`**: Initializes the ROS node with the name `pose_subscriber`.
- **`ros::NodeHandle`**: Manages communication with ROS.
- **`n.subscribe`**: Subscribes to `/turtle1/pose` (message type: `turtlesim::Pose`) with a queue size of 10. Links incoming messages to `poseCallback`.
- **`ros::spin`**: Keeps the node alive, waiting for messages.

---

## **Steps to Implement a Subscriber**

1. **Initialize the ROS Node**:
   - Use `ros::init` to set up the node.

2. **Subscribe to a Topic**:
   - Use `NodeHandle.subscribe` to connect to a topic and specify a callback function.

3. **Wait for Messages**:
   - Use `ros::spin()` to keep the program running and process incoming messages.

---

## **Output**

- Whenever the turtle's pose updates, the callback function prints the position to the console:
  ```plaintext
  Turtle pose: x:5.544445, y:5.544445
  ```

---

## **Summary**

- This subscriber node connects to the `/turtle1/pose` topic to receive position updates from the TurtleSim simulation.
- It processes incoming messages using the `poseCallback` function and displays the turtle's coordinates.


# ROS Subscriber in Python

This guide demonstrates a complete implementation of a ROS subscriber node in Python that subscribes to the `/turtle1/pose` topic and processes incoming messages.

---

## **Python Code**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    # Log the turtle's pose (x, y coordinates)
    rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)

def pose_subscriber():
    # Initialize the ROS node
    rospy.init_node('pose_subscriber', anonymous=True)

    # Create a subscriber to the /turtle1/pose topic
    rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

    # Wait and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_subscriber()
    except rospy.ROSInterruptException:
        pass
```

---

## **Explanation**

### **1. Import Required Libraries**
- **`rospy`**: The ROS Python client library.
- **`turtlesim.msg.Pose`**: The message type for the turtle's pose, including position and orientation.

### **2. Callback Function**
```python
def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
```
- Called whenever a message is received on `/turtle1/pose`.
- Logs the `x` and `y` coordinates of the turtle.

### **3. Subscriber Setup**
```python
def pose_subscriber():
    rospy.init_node('pose_subscriber', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
    rospy.spin()
```
- **`rospy.init_node`**: Initializes the ROS node with the name `pose_subscriber`.
- **`rospy.Subscriber`**: Subscribes to `/turtle1/pose`, associates incoming messages with `poseCallback`.
- **`rospy.spin`**: Keeps the program alive and processes incoming messages.

### **4. Graceful Shutdown**
```python
if __name__ == '__main__':
    try:
        pose_subscriber()
    except rospy.ROSInterruptException:
        pass
```
- Handles interruptions (e.g., Ctrl+C) gracefully.

---

## **Summary**

This Python subscriber node:
- Subscribes to the `/turtle1/pose` topic.
- Processes incoming `Pose` messages.
- Logs the turtle's `x` and `y` coordinates.

The script is designed to work with the TurtleSim simulation.
