# Creating and Building a ROS Package

This guide explains how to create and build a ROS package within a catkin workspace.

---

## 1. Command Syntax
```bash
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
### Explanation:
- **`<package_name>`**: The name of the package you want to create.
- **`[depend1] [depend2] [depend3]`**: Optional dependencies required by your package (e.g., `std_msgs`, `rospy`, `roscpp`).

---

## 2. Creating a New Package

Run the following commands:

```bash
cd ~/catkin_ws/src
catkin_create_pkg test_pkg std_msgs rospy roscpp
```

### Explanation:
- **Step 1**: Navigate to the `src` directory of your catkin workspace.
  ```bash
  cd ~/catkin_ws/src
  ```
- **Step 2**: Use `catkin_create_pkg` to create a new package named `test_pkg` with dependencies:
  - `std_msgs`: Standard ROS message types.
  - `rospy`: Python ROS client library.
  - `roscpp`: C++ ROS client library.
- **Result**: A new folder named `test_pkg` will be created in the `src` directory containing:
  - `CMakeLists.txt`: Build instructions.
  - `package.xml`: Package description and dependencies.

---

## 3. Building the Package

Run the following commands:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Explanation:
- **Step 1**: Navigate to the root of your catkin workspace:
  ```bash
  cd ~/catkin_ws
  ```
- **Step 2**: Build the workspace using:
  ```bash
  catkin_make
  ```
  - This compiles the newly created package and links dependencies.
- **Step 3**: Source the workspace environment:
  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```
  - This ensures ROS recognizes the new package.

---

## 4. Important Notes

- **Unique Package Names**:
  - Within the same workspace: **Duplicate package names are not allowed.**
  - Across different workspaces: **Duplicate package names are allowed.**
- This ensures ROS can uniquely identify packages when resolving dependencies or running nodes.

---

## Summary

This process includes:
1. Creating a ROS package using `catkin_create_pkg`.
2. Compiling the workspace with `catkin_make`.
3. Configuring the environment with `source`. 
