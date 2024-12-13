# Creating a Custom ROS Message

This guide explains how to define and use a custom message type in ROS by creating a `.msg` file, integrating it into a package, and generating the necessary files.

---

## **1. Define a `.msg` File**

### Example: `Person.msg`
```plaintext
string name
uint8 sex
uint8 age

uint8 unknown=0
uint8 male=1
uint8 female=2
```
- **Fields**:
  - `string name`: A string for the person's name.
  - `uint8 sex`: A byte (integer) to represent the person's sex.
  - `uint8 age`: A byte (integer) to represent the person's age.

- **Constants**:
  - `uint8 unknown=0`: Represents an unknown sex.
  - `uint8 male=1`: Represents male sex.
  - `uint8 female=2`: Represents female sex.

This `.msg` file defines the structure of the custom message `Person`.

---

## **2. Update `package.xml`**

Add the necessary dependencies for message generation and runtime:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

- **`message_generation`**: Used during the build process to generate the required files for the custom message.
- **`message_runtime`**: Provides the runtime support needed to use the generated message files.

---

## **3. Update `CMakeLists.txt`**

### Add Required Packages
```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
```

### Specify the Message Files
```cmake
add_message_files(
  FILES
  Person.msg
)
```
- Registers the `Person.msg` file for message generation.

### Generate Messages
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
- Generates the necessary files for the `Person` message.
- Declares dependencies like `std_msgs` for field types if used.

### Catkin Package
```cmake
catkin_package(
  CATKIN_DEPENDS message_runtime
)
```
- Ensures that the package depends on `message_runtime` for runtime support.

---

## **4. Build the Package**

After making the changes:
1. Build the package:
   ```bash
   catkin_make
   ```
2. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

---

## **5. Generated Files**

After building, ROS generates:
- **C++ Header Files**: Found in `devel/include/<package_name>`.
- **Python Modules**: Found in `devel/lib/pythonX/dist-packages/<package_name>`.

These files can now be used in your ROS nodes to publish or subscribe to the `Person` message type.

---

## **Summary**
This workflow includes:
1. Defining a custom message (`Person.msg`).
2. Adding dependencies in `package.xml`.
3. Configuring `CMakeLists.txt` for message generation.
4. Building the package to generate files for using the message in ROS nodes.

By following these steps, you can create and integrate custom messages into your ROS workflow.
