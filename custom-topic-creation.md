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


# person_publisher.cpp

This file demonstrates a ROS publisher node in C++ that publishes custom messages of type `learning_topic::Person` to the `/person_info` topic.

---

## **Code**

```cpp
#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "person_publisher");

    // Create a NodeHandle
    ros::NodeHandle n;

    // Create a Publisher to publish to /person_info topic with the message type learning_topic::Person
    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);

    // Set the loop frequency
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // Initialize a message of type learning_topic::Person
        learning_topic::Person person_msg;
        person_msg.name = "Tom";
        person_msg.age = 18;
        person_msg.sex = learning_topic::Person::male;

        // Publish the message
        person_info_pub.publish(person_msg);

        ROS_INFO("Publish Person Info: name:%s age:%d sex:%d",
                 person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // Sleep for the remainder of the loop cycle
        loop_rate.sleep();
    }

    return 0;
}
```

---

## **Explanation**

### **1. Includes**
- **`#include <ros/ros.h>`**: ROS core library for C++.
- **`#include "learning_topic/Person.h"`**: Header file for the custom message type `Person`.

### **2. Node Initialization**
```cpp
ros::init(argc, argv, "person_publisher");
```
- Initializes the ROS node with the name `person_publisher`.

### **3. Create a NodeHandle**
```cpp
ros::NodeHandle n;
```
- Facilitates communication with the ROS system.

### **4. Create a Publisher**
```cpp
ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);
```
- Publishes messages to the `/person_info` topic.
- Message type: `learning_topic::Person`.
- Queue size: 10.

### **5. Message Initialization**
```cpp
learning_topic::Person person_msg;
person_msg.name = "Tom";
person_msg.age = 18;
person_msg.sex = learning_topic::Person::male;
```
- Creates and populates a `Person` message.
  - `name`: Name of the person.
  - `age`: Age of the person.
  - `sex`: Sex of the person, using constants defined in the `Person` message (`male`).

### **6. Publish the Message**
```cpp
person_info_pub.publish(person_msg);
```
- Sends the message to the `/person_info` topic.

### **7. Log the Published Data**
```cpp
ROS_INFO("Publish Person Info: name:%s age:%d sex:%d",
         person_msg.name.c_str(), person_msg.age, person_msg.sex);
```
- Logs the message details to the console.

### **8. Loop Frequency**
```cpp
ros::Rate loop_rate(1);
```
- Sets the loop rate to 1 Hz, ensuring the message is published once per second.

### **9. Keep the Node Running**
```cpp
while (ros::ok()) {
    ...
}
```
- Continuously publishes messages as long as the ROS system is running.

---

## **Summary**
- This publisher node sends `Person` messages to the `/person_info` topic at 1 Hz.
- Each message contains details about a person (`name`, `age`, and `sex`).
- The node logs the published message information for debugging and monitoring purposes.


# person_subscriber.cpp

This file demonstrates a ROS subscriber node in C++ that subscribes to the `/person_info` topic and processes messages of type `learning_topic::Person`.

---

## **Code**

```cpp
/**
 * Example ROS subscriber node for /person_info topic, subscribing to messages of type learning_topic::Person
 */

#include <ros/ros.h>
#include "learning_topic/Person.h"

// Callback function to process received messages
void personInfoCallback(const learning_topic::Person::ConstPtr& msg)
{
    // Print the received message information
    ROS_INFO("Subscribe Person Info: name:%s age:%d sex:%d",
             msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "person_subscriber");

    // Create a NodeHandle
    ros::NodeHandle n;

    // Create a subscriber to the /person_info topic
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // Keep the program running and process incoming messages
    ros::spin();

    return 0;
}
```

---

## **Explanation**

### **1. Includes**
- **`#include <ros/ros.h>`**: ROS core library for C++.
- **`#include "learning_topic/Person.h"`**: Header file for the custom message type `Person`.

### **2. Callback Function**
```cpp
void personInfoCallback(const learning_topic::Person::ConstPtr& msg)
{
    ROS_INFO("Subscribe Person Info: name:%s age:%d sex:%d",
             msg->name.c_str(), msg->age, msg->sex);
}
```
- Processes messages received on the `/person_info` topic.
- Extracts and logs the `name`, `age`, and `sex` fields from the `Person` message.

### **3. Node Initialization**
```cpp
ros::init(argc, argv, "person_subscriber");
```
- Initializes the ROS node with the name `person_subscriber`.

### **4. Create a NodeHandle**
```cpp
ros::NodeHandle n;
```
- Facilitates communication with the ROS system.

### **5. Create a Subscriber**
```cpp
ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);
```
- Subscribes to the `/person_info` topic.
- Message type: `learning_topic::Person`.
- Queue size: 10.
- Associates incoming messages with the `personInfoCallback` function.

### **6. Process Incoming Messages**
```cpp
ros::spin();
```
- Keeps the program running and processes messages as they arrive.

---

## **Summary**
- This subscriber node listens to the `/person_info` topic.
- When a message is received, the `personInfoCallback` function processes and logs the `name`, `age`, and `sex` fields.
- It runs continuously, waiting for incoming messages until the program is terminated.

