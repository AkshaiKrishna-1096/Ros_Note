# Ros Note - Communication Basics & Package Enhancement
* Akshai Krishna KP (CS24B1096)

---
## 🔄 Difference Between ROS 1 and ROS 2
---

| Feature                    | ROS 1                                     | ROS 2                                             |
|----------------------------|-------------------------------------------|---------------------------------------------------|
| 🧠 Architecture            | Centralized (`roscore` needed)            | Decentralized (no central master)                 |
| 📡 Communication Middleware| Custom TCP/UDP via `ros_comm`             | DDS (Data Distribution Service)                   |
| 🌐 Discovery Mechanism     | Centralized topic/service registration    | Peer-to-peer DDS discovery                        |
| 🛠 Real-time Support       | Very limited                              | Real-time capable (PREEMPT-RT support)            |
| 🏎 Performance Tuning      | No QoS                                    | Full QoS support (reliability, durability, etc.)  |
| 🔒 Security                | No built-in security                      | DDS-Security (encryption, access control)         |
| 🚦 Multi-Robot Support     | Difficult to scale                        | Native support via DDS                            |
| 🖥 OS Support              | Linux only (mainly Ubuntu)                | Linux, Windows, macOS, RTOS                       |
| 📦 Build System            | `catkin` (CMake-based)                    | `ament` (modular, extensible)                     |
| 🚀 Launch System           | XML-based launch files                    | Python-based `launch_ros`                         |
| 📚 Language Support        | C++03 & Python2                           | C++17 & Python3                                   |

> The above table only provide some basic info about difference between `ROS1` and `ROS2`.
> For more information, visit [Changes between ROS 1 and ROS 2](https://design.ros2.org/articles/changes.html)

---
## ROS communication basics
---

### ❓ What are DDS and Peer-to-Peer Communication?

**Peer-to-peer (P2P) communication** means that each node acts as a **peer** capable of directly communicating with other nodes **without relying on a central server**. Each node can function as both a **client** and a **server**, enabling decentralized and distributed communication.

**DDS (Data Distribution Service)** is a **publish-subscribe communication middleware** that automatically discovers and connects nodes without a centralized server. It forms a **scalable**, **low-latency**, and **decentralized system**, solving the single point of failure problem.

DDS handles **communication logic** under the hood using **peer-to-peer logic**.

---

### ❓ Why Did ROS 2 Drop the ROS Master?

In **ROS 1**, the **ROS Master** served as a central agent to manage communication between nodes. However, this led to a **single point of failure**—if the master crashed, the entire system could become non-functional.

**ROS 2** eliminated the master by adopting **DDS**, which enables **decentralized communication** through automatic node discovery and **peer-to-peer messaging**.

This change improves:
- ✅ **Scalability**
- ✅ **Fault tolerance**
- ✅ **Real-time performance**
- ✅ Ability to configure **Quality of Service (QoS)**


DDS provides **QoS (Quality of Service)** controls, allowing ROS 2 developers to:

- Choose between **reliable** and **best-effort** delivery
- Set **latency budgets** and **lifespan** of messages
- Control **history** and **durability** for late-joining nodes

These features are essential for making **highly capable Robot**. 

---

## ROS Launch file and QoS
---

The next task is to make a launch file into the workspace and add QoS into node.

### 🚀 Launch File

The **ROS 2 Launch system** configures the system to specify:
- What programs to run
- Where to run them
- What arguments to pass
- ROS-specific conventions for reusability (like remapping topics or namespaces)

For more information, refer to the official [ROS 2 Humble Launch File documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

**▶️ How to Run the Launch File**

Make sure you have:
1. Cloned the repository
2. Built the workspace using `colcon build`
3. Sourced the environment

Then, run:

```bash
ros2 launch launch_week1 mars.launch.py
```

> **NOTE**
> I have made the launch file in python for my ease. We can make it in XML and YAML also.

---
### 📡 QoS (Quality of Service) in ROS 2
---
QoS allow us to tune the communication between the ROS2. It can be reliable as TCP and best-effort as UDP depending on the QoS profile we set.\

**⚙️ Common QoS Policies**

- **History** – `Keep last` or `Keep all` messages
- **Depth** – Size of the message queue
- **Reliability** – `RELIABLE` or `BEST_EFFORT`
- **Durability** – `TRANSIENT_LOCAL` or `VOLATILE`
- **Deadline** – Expected interval between messages
- **Lifespan** – Duration a message remains valid
- **Liveliness** – `AUTOMATIC` or `MANUAL_BY_TOPIC`
- **Lease Duration** – Time within which liveliness must be asserted

*The QoS also ensure that the connection between the nodes are only made if they have compatible QoS profile.*\
For more information visit [Quality of Services](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

**QoS profile**, i have used for the node is Reliable, volatile and queuesize of 10. because as i know the node is internally controlled and not sensor-based for now.
```C
    // setting up QoS profile of the node
    rclcpp::QoS qos_profile(10);        // Depth : keep last 10 msg.
    qos_profile.reliable();             // Ensure all the msg are sent.
    qos_profile.durability_volatile();  // Do not persist msg.
```

---
> **NOTE**
> For the info about the previous task, checkout the task_log directory.
