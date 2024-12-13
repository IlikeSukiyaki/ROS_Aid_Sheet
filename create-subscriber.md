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
