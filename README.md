# 🤖 ROS2 Note – Week 1: Publisher & Subscriber Nodes

- *Akshai Krishna KP (CS24B1096)*
- Welcome to **Week 1** of the ROS2 learning journey! This task focuses on getting hands-on with ROS2 by creating a basic **publisher** and **subscriber** node interaction. 🚀

---

## 📌 Goal

> ✅ **Objective**: Create two ROS2 nodes:
> - One node to **publish numbers** on a topic named `/numbers`.
> - Another node to **subscribe** to `/numbers`, **square the number**, and print the result.

---

## 📅 Progress Log

### 🗓️ 12 May 2025
- Exploring the [Articulated Robotics tutorials](https://articulatedrobotics.xyz/tutorials/) and the [official ROS2 documentation](https://docs.ros.org/en/humble/index.html) provided by MaRs Club.
- Switched to video-based learning on YouTube for better understanding.

### 🗓️ 13–15 May 2025
- Followed the [ROS2 YouTube playlist](https://youtu.be/0aPbWsyENA8) (Python-based).
- Cross-referenced the tutorials in documentation for better clarity.

### 🗓️ 16 May 2025
- Began implementing the nodes in C++.
- Encountered errors, refer to documentation for solution.
- Tried to learn **OOP** for C++.

---

## 📚 Learning Outcome Overview

> ✨ *This section summarizes what i learned in this week about ROS2 humble.*

---

### ✅ What I Learned

1. 🛠️ **Installed and Set Up ROS 2 Humble**
   - Followed the [official installation guide](https://docs.ros.org/en/humble/Installation.html) for ROS 2 Humble on Ubuntu 22.04.

2. 🧱 **Created and Configured a ROS 2 Workspace**
   - Learned how to organize ROS 2 packages inside a workspace:
     ```bash
     mkdir -p ~/ros_ws/src
     cd ~/ros_ws
     colcon build
     source install/setup.bash
     ```
   - Reference: [Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

3. 📦 **Created Custom ROS 2 Packages**
   - Created a package using:
     ```bash
     ros2 pkg create --build-type ament_cmake week1
     ```
   - Added dependencies in `package.xml` and `CMakeLists.txt`.

4. 📡 **Created a Publisher and Subscriber Node in C++**
   - Wrote a C++ node.
   - Used `rclcpp::Publisher` and `std_msgs::msg::Int32`.
   - Handle callback using `std::bind`
   - Reference: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

5. 🐍 **Created a Publisher and Subscriber Node in Python**
   - Developed a Python node using `rclpy` to publish data.
   - Used `rclpy.Publisher` and `std_msgs.msg.Int32`.
   - Reference: [Python Publisher/Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

6. 🔧 **Managed Dependencies and Run Nodes**
   - Added required dependencies in `package.xml`:
     ```xml
     <depend>rclcpp</depend>
     <depend>std_msgs</depend>
     ```
   - Registered nodes in `CMakeLists.txt`.
   - Ran the nodes.
---

## 🧠 ROS Communication Overview

📊 The ROS graph for this setup:

![rqt_graph](image/rosgraph.svg)

### 🔄 Node Communication

| Node Name           | Role           | Topic Used | Message Type         |
|---------------------|----------------|------------|-----------------------|
| `number_publisher`  | Publisher      | `/numbers` | `std_msgs::msg::Int32` |
| `squared_publisher` | Subscriber → Publisher | `/numbers` (sub) → prints squared value | `std_msgs::msg::Int32` |

- `number_publisher` publishes integers on the topic `/numbers`.
- `squared_publisher` subscribes to `/numbers`, squares the value, and logs the output using `RCLCPP_INFO`.

---

## ⚙️ Dependencies Setup

Ensure the following dependencies are added:

- In `package.xml`:  
  Add `<depend>rclcpp</depend>` and `<depend>std_msgs</depend>`

- In `CMakeLists.txt`:  
  Link the dependencies and add executables for both nodes using `ament_target_dependencies()`.

---

## 🚀 How to Build & Run the Nodes

Follow these simple steps to set up and run the project on your local machine:

---

### 🔧 1. Install ROS2 Humble

🛠️ [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

Before building your workspace, source the ROS2 setup script in the terminal:
```Bash
source /opt/ros/humble/setup.bash
```
>⚠️ If you want this to be automatic for every new terminal, add it to your `.bashrc` with:
> ```Bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
>source ~/.bashrc
>```


Also, ensure `colcon` build is installed:
```Bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```
---

### 📁 2. Create Your Workspace

```Bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
---

### 3. clone the git-repo inside ros_ws
```Bash
git clone https://github.com/AkshaiKrishna-1096/Ros_Note.git
```
---

### 4. Build the workspace
```Bash
cd ~/ros_ws
colcon build
```
---

### 5. now source the Workspace overlay 
```Bash
source ~/ros_ws/install/setup.bash
```
>⚠️Remeber:Source this in every terminal you use ROS command in!
---

### 6. Open two terminal and run the following
**Terminal 1**
``` Bash
ros2 run week1 number_publisher
```
**Terminal 2**
```
ros2 run week1 squared_publisher
```
---

### Optinal: Visualize the Graph
To see the communication graphically:\
In another terminal
```Bash
rqt_graph
```
---
## What to do next 
> 🚀 Now next step is to learn about services and parameters in ROS2.
