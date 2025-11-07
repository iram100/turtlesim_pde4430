
# ROS2 Turtlesim Simulation

This repository contains tasks completed as part of the PDE4430 using ROS2 Jazzy

---

##  **Task 1: Discoveries**

All answers are included in the file:  
**[`Task1_answers.txt`](./Task1_answers.txt)**

### Commands Executed

- `ros2 node list` — List all running nodes  
- `ros2 topic list` — Display all active topics  
- `ros2 topic info /turtle1/cmd_vel` — Identify publisher node and message type  
- `ros2 topic info /turtle1/pose` — Inspect the pose message type  
- `ros2 topic echo /turtle1/color_sensor` — View color sensor messages  
- `ros2 topic hz /turtle1/cmd_vel` — Measure message publication frequency  

---

##  **Task 2: Autonomous Movement**

### Nodes Implemented

 **`straight_line`** — Publishes constant forward velocity to `/turtle1/cmd_vel`  
   
  ```bash
  ros2 run turtlesim_pde4430 straight_line
````

 **`circle`** — Publishes linear + angular velocity to move in a circular path
  

  ```bash
  ros2 run turtlesim_pde4430 circle
  ```

* **`figure8`** — Approximates a figure-eight trajectory
  
  ```bash
  ros2 run turtlesim_pde4430 figure8
  ```

* **`roomba`** — Simulates a simple random-bounce cleaner using `/turtle1/pose` feedback


  ```bash
  ros2 run turtlesim_pde4430 roomba
  ```

* **`spawn_multi`** — Spawns extra turtles and publishes motion commands to them
 

  ```bash
  ros2 run turtlesim_pde4430 spawn_multi
  ```

* **`goto`** — Go-to-goal controller
  

  ```bash
  ros2 run turtlesim_pde4430 goto --x 8.0 --y 3.0
  ```

###  Launch File

Path:

```
src/turtlesim_pde4430/launch/turtlesim_demo_launch.py
```

### To Start the Demo

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlesim_pde4430 turtlesim_demo_launch.py
```

---

##  **Task 3: User Input and Advanced Interaction**

This task demonstrates **two ways to control the turtle’s speed** at runtime:

1. Interactive user input
2. Dynamic ROS2 parameters

All evidence and output logs are stored in:
 `src/turtlesim_pde4430/task3_logs/`

---

###  **A. Interactive Prompt — `user_drive_prompt.py`**

**Run Command:**

```bash
python3 turtlesim_pde4430/user_drive_prompt.py
```

**User Input Example:**

```
Enter linear speed (m/s) [1.0]: 0.5
Enter angular speed (rad/s) [0.0]: 0.3
```

**Logged Output:**

```
[user_drive_prompt]: Using linear=0.5 m/s, angular=0.3 rad/s
```

**Captured `/turtle1/cmd_vel` Sample (Prompt Run):**

```
linear: x=1.0 y=0.0 z=0.0
angular: x=0.0 y=0.0 z=0.0
```



**Evidence Files:**

* `task3_logs/prompt_cmdvel.txt` — Captured Twist message sample

---

###  **B. ROS2 Parameters — `user_drive_params.py`**

**Run Command:**

```bash
python3 turtlesim_pde4430/user_drive_params.py &
```

**Parameter Updates:**

```bash
ros2 param set /user_drive_params linear_speed 0.7
ros2 param set /user_drive_params angular_speed 0.2
```

**Recorded Parameter Values:**

```
Double value is: 0.7
Double value is: 0.2
```

**Captured `/turtle1/cmd_vel` Sample (Params Run):**

```
linear: x=1.0 y=0.0 z=0.0
angular: x=0.0 y=0.0 z=0.0
```

**Evidence Files:**

* `task3_logs/params_linear.txt`
* `task3_logs/params_angular.txt`
* `task3_logs/params_cmdvel.txt`

---




**Summary:**

* **Task 1:** ROS2 Turtlesim exploration and topic discovery
* **Task 2:** Implementation of multiple autonomous movement patterns
* **Task 3:** Interactive and parameter-driven control of turtle motion
* **Environment:** ROS2 Jazzy on Ubuntu 24.04 LTS

```

