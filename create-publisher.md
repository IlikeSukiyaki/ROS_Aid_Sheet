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
