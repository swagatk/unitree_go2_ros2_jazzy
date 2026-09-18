# Unitree Go2 Quadruped Simulation & 2D SLAM Lab Tutorial

**Target System:** ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS (WSL2 / Native Linux)

**Simulator:** Gazebo Harmonic (GZ Sim)

**Estimated Time:** 120 Minutes (2 Hours)

**Author / Course:** Autonomous Robotics & Quadruped Systems Lab

## Lab Overview & Objectives

In this hands-on laboratory session, you will configure, operate, and map an environment using the Unitree Go2 quadruped robot in simulation. By the end of this session, you will be able to:

1. Build and configure the Unitree Go2 ROS 2 Jazzy simulation workspace with Gazebo Harmonic.

2. Resolve multi-publisher hardware interface and bridge conflicts on `/joint_states` and `/tf`.

3. Overcome WSL2 build resource limitations (linker segfaults) and graphical display quirks.

4. Bridge 3D LiDAR point clouds to 2D laser scans with synthesized intensity channels for RViz2.

5. Deploy a synchronized odometry transform broadcaster to eliminate quadruped mesh dismemberment.

6. Tune and deploy `slam_toolbox` to construct and export a crisp 2D occupancy grid map.

### Schedule Breakdown

| **Phase** | **Activity** | **Duration** | 
| **Phase 1** | Package Installation, Dependency Management & Launch File Patching | 40 Mins | 
| **Phase 2** | Simulation Launch, Teleoperation & Sensor Visualization | 40 Mins | 
| **Phase 3** | Transform Synchronization, 2D SLAM Mapping & Map Export | 40 Mins | 

## Phase 1: Installing the Unitree ROS 2 Package (40 Mins)

### 1.1 Prerequisites & System Dependencies

Ensure your ROS 2 Jazzy installation is updated and all necessary controller packages, CLI plugins, and Python dependencies are installed. Open a terminal inside Ubuntu 24.04:

```
# 1. Verify ROS 2 distribution
echo $ROS_DISTRO  # Must print 'jazzy'

# 2. Deactivate Conda if active to avoid Python library hijacking
conda deactivate 2>/dev/null || true

# 3. Update repositories and install all simulation, control, and mapping packages
sudo apt update && sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-yaml \
  binutils-gold \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gazebo-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2controlcli \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-effort-controllers \
  ros-jazzy-position-controllers \
  ros-jazzy-velocity-controllers \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-velodyne \
  ros-jazzy-velodyne-description \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-map-server \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-tf2-tools

```

Initialize `rosdep` if not already completed:

```
sudo rosdep init 2>/dev/null || true
rosdep update

```

### 1.2 Workspace Creation and Repository Cloning

We will clone the **RobInLabUJI Unitree Go2 Gazebo Harmonic Simulation package** (which includes Gazebo models and CHAMP kinematics):

```
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src

# Remove any conflicting hardware SDK packages if previously cloned
rm -rf ~/go2_ws/src/unitree_go2_ros2_jazzy

# Clone the official ROS 2 Jazzy Gazebo Harmonic simulation repository
git clone https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy.git ~/go2_ws/src/unitree_go2_ros2_jazzy

```

Verify that the expected packages are present:

```
ls ~/go2_ws/src/unitree_go2_ros2_jazzy
# Expected output: champ  champ_base  champ_msgs  README.md  slam_scripts  unitree_go2_description  unitree_go2_sim

```

The `slam_scripts` folder is a self-contained `ament_python` ROS 2 package (it ships its own `package.xml`/`setup.py`), so it builds and installs alongside the other packages without any special handling.

### 1.3 Patching the Simulation Launch File (Critical Stability Step)

In ROS 2 Jazzy with Gazebo Harmonic, several default configurations cause severe TF collisions and robot dismemberment:

* Duplicate `/joint_states` publishing creates race conditions between Gazebo and ROS 2 control.

* Bidirectional `/tf` bridging loops coordinate frames back and forth with clock latency.

* Redundant static and EKF nodes fight over `base_footprint` and freeze linear odometry at $(0, 0, 0)$.

Open the primary launch script in an editor:

```
nano ~/go2_ws/src/unitree_go2_ros2_jazzy/unitree_go2_sim/launch/unitree_go2_launch.py

```

Perform the following modifications:

1. **Disable Internal Gazebo Joint State Publishing (Line \~97):**

   ```
   # Change True to False:
   {"publish_joint_states": False},
   
   ```

2. **Comment out the `/joint_states` Bridge Argument (Line \~242):**

   ```
   # '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
   
   ```

3. **Comment out the `/tf` Bridge Argument:**
   Search for `/tf` within the bridge arguments list (around lines 230–250) and comment it out:

   ```
   # '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
   
   ```

4. **Disable Conflicting EKF Nodes:**
   Locate `base_to_footprint_ekf` and `footprint_to_odom_ekf` (around lines 100–140). Comment out their definitions:

   ```
   # base_to_footprint_ekf = Node(...)
   # footprint_to_odom_ekf = Node(...)
   
   ```

5. **Clean up Node Invocations:**
   Scroll to the bottom of the file inside `LaunchDescription([...])`. Comment out any references to:

   ```
   # base_to_footprint_ekf,
   # footprint_to_odom_ekf,
   
   ```

Save and exit (`Ctrl+O` $\rightarrow$ `Enter` $\rightarrow$ `Ctrl+X`).

*(Perform identical edits on `unitree_go2_launch_TI.py` if planning to map the indoor TI building world).*

### 1.4 Compiling the Workspace (WSL2 Memory Optimization)

To prevent the GNU linker (`ld`) from running out of RAM and crashing with `Signal 11 [Segmentation fault]` during the `champ_base` compilation, build `champ_base` with a single worker before compiling the rest:

```
cd ~/go2_ws

# 1. Resolve all dependencies
rosdep install --from-paths src --ignore-src -r -y

# 2. Compile champ_base first with 1 parallel worker to conserve memory
colcon build --symlink-install --packages-select champ_base --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Build the remainder of the workspace
colcon build --symlink-install

# 4. Source the built environment
source install/setup.bash
echo "source ~/go2_ws/install/setup.bash" >> ~/.bashrc

```

### Phase 1 Student Tasks

1. **Package Verification:** Run `ros2 pkg list | grep -E "go2|champ"` and list all packages detected by your ROS 2 environment.

2. **Dynamic URDF Model Inspection:**
   Execute the following snippet to locate and parse the robot description:

   ```
   MODEL_FILE=$(find ~/go2_ws/src/unitree_go2_ros2_jazzy/unitree_go2_description -name "*.xacro" -o -name "*.urdf" | head -n 1)
   echo "Using model file: $MODEL_FILE"
   ros2 run xacro xacro "$MODEL_FILE" > /tmp/go2.urdf
   grep -E "<link name=\"(base_link|base_footprint|trunk)\"" /tmp/go2.urdf
   
   ```

   Record the root links of the quadruped.

3. **Conceptual Question:** Why does bridging `/tf` bidirectionally between Gazebo Harmonic and ROS 2 cause robot bodies to jitter and tear apart in RViz?

## Phase 2: Teleoperation & Sensor Visualization (40 Mins)

The Unitree Go2 publishes a 3D LiDAR point cloud (`/velodyne_points/points`). Standard 2D SLAM tools require a 2D `sensor_msgs/msg/LaserScan`. Furthermore, RViz2 requires valid non-zero intensities to properly execute color transforming.

Both helper nodes described below already ship as installable ROS 2 executables inside the `slam_scripts` package (`unitree_go2_ros2_jazzy/slam_scripts`), so there is no need to hand-copy scripts to your home directory. Building the workspace once (Section 1.4) makes them available via `ros2 run`.

### 2.1 Cloud-to-LaserScan Converter Node

The node lives at `slam_scripts/slam_scripts/cloud_to_scan.py` and is registered as the `cloud_to_scan` console script. It slices the 3D cloud into a horizontal 2D laser scan with synthetic intensity data, filtering ground reflections (`z < -0.08m`) and ceiling hits (`z > 1.2m`) before publishing to `/scan`.

### 2.2 Synchronized Odometry TF Broadcaster

In Gazebo, `/odom` publishes vehicle displacement, but the dynamic transform link between `odom` and `base_footprint` must track live simulation clock stamps to prevent transform extrapolation errors. This is handled by `slam_scripts/slam_scripts/odom_to_tf.py`, registered as the `odom_to_tf` console script, which re-broadcasts `/odom` as a live `odom -> base_footprint` TF using a best-effort QoS profile matched to Gazebo's publisher.

### 2.3 Running the Simulation & WSL2 Display Handling

Open separate terminal tabs for each process:

**Terminal 1: Gazebo Simulation Launch**

```
source ~/go2_ws/install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_launch.py

```

*(Note for WSL2: If the Gazebo 3D GUI window does not appear automatically, Gazebo is running in headless mode in the background. You can open the GUI frontend in another terminal with `gz sim -g` or rely directly on RViz2, which uses significantly less GPU overhead).*

**Terminal 2: PointCloud to LaserScan Converter**

```
source ~/go2_ws/install/setup.bash
ros2 run slam_scripts cloud_to_scan --ros-args -p use_sim_time:=true

```

**Terminal 3: Odometry Transform Broadcaster**

```
source ~/go2_ws/install/setup.bash
ros2 run slam_scripts odom_to_tf --ros-args -p use_sim_time:=true

```

**Terminal 4: Keyboard Teleoperation**

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**Terminal 5: RViz2 Visualization**
*(If not already launched by the launch script)*

```
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2

```

### 2.4 Configuring RViz2

In the RViz2 left **Displays** panel:

1. Under **Global Options**, verify `Fixed Frame` is set to `odom`.

2. Ensure **RobotModel** is checked (`Description Topic: /robot_description`).

3. Add **LaserScan** (`Topic: /scan`, `Size: 0.05m`, `Color Transformer: AxisColor` or `FlatColor`).

4. Add **Image** (`Topic: /rgb_image`) to inspect the forward camera.

### Phase 2 Student Tasks

1. **Publisher Verification:** Run `ros2 topic info /joint_states` and `ros2 topic info /tf`. Verify that:

   * `/joint_states` has **exactly 1 publisher** (`joint_states_controller`).

   * `/tf` has **no duplicate bridges**.

2. **Live Odometry Verification:** Move the Go2 forward using teleop keys (`i`). Run:

   ```
   ros2 run tf2_ros tf2_echo odom base_footprint
   
   ```

   Confirm that the `Translation: [X, Y, Z]` values continuously change and do not remain stuck at `0.0`.

## Phase 3: 2D Mapping with SLAM Toolbox (40 Mins)

### 3.1 SLAM Parameter Configuration

Quadrupeds exhibit slight body rocking during locomotion. We configure `slam_toolbox` with reduced distance/angle travel thresholds so scan matching updates immediately even during low-speed trotting. These parameters are already tracked in the repo at `slam_scripts/config/go2_slam.yaml` and are installed to `share/slam_scripts/config/go2_slam.yaml`, so no manual file creation is required:

```
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # Solver Plugin
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT

    # Frames
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan

    # Motion Thresholds (Sensitive motion integration for quadrupeds)
    mode: mapping
    map_update_interval: 0.5
    minimum_time_interval: 0.2
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1

    # Scan Matcher Bounds
    resolution: 0.05
    max_laser_range: 20.0
    minimum_laser_range: 0.25
    use_scan_matching: true
    use_scan_barycenter: true
    distance_penalty_factor: 0.5
    angle_penalty_factor: 0.5

    # Graph Constraints
    transform_timeout: 0.5
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: false

```

If you want to tweak a parameter, edit `~/go2_ws/src/unitree_go2_ros2_jazzy/slam_scripts/config/go2_slam.yaml` and rebuild (`colcon build --packages-select slam_scripts --symlink-install`); with `--symlink-install` the installed copy updates immediately without rebuilding.

### 3.2 Launching SLAM Toolbox

Open a new terminal and launch the asynchronous mapping node, pointing `params_file` at the config installed by `slam_scripts`:

```
source ~/go2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  params_file:=$(ros2 pkg prefix slam_scripts)/share/slam_scripts/config/go2_slam.yaml

```

**Optional one-shot bringup:** Terminals 2, 3, and this SLAM Toolbox launch can be combined into a single command using the bundled launch file:

```
source ~/go2_ws/install/setup.bash
ros2 launch slam_scripts slam_bringup.launch.py use_sim_time:=true

```

### 3.3 Active Mapping in RViz2

Switch to your RViz2 window:

1. **Change Fixed Frame to `map`:** Under **Global Options**, click `Fixed Frame` and select **`map`** (Critical: if left as `odom`, the map will fail to render progressive updates).

2. Click **Add** $\rightarrow$ select **Map** $\rightarrow$ set `Topic` to **`/map`**.

3. Set **Durability Policy** to `Transient Local`.

4. Click **Reset** in the bottom-left corner of RViz.

Drive the robot around the colored obstacles using the teleop terminal. The white corridors of free space and sharp black boundary walls will expand outward as the robot explores.

### 3.4 Saving the Completed Map

Once the environment has been fully circumnavigated, export the occupancy grid to disk:

```
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/go2_sim_map --ros-args -p use_sim_time:=true

```

Verify that the map files were written:

```
ls -lh ~/go2_sim_map.*
# Output:
# ~/go2_sim_map.pgm (Occupancy grid image)
# ~/go2_sim_map.yaml (Origin, resolution, and threshold metadata)

```

### Phase 3 Student Tasks

1. **Full Exploration:** Drive the Go2 in a closed loop around the red cube, blue cube, yellow cylinder, and green box until the map shows closed boundaries without open gaps.

2. **Transform Hierarchy Audit:** Generate a PDF snapshot of the verified TF tree:

   ```
   ros2 run tf2_tools view_frames
   
   ```

   Open `frames.pdf` and verify that the complete chain links cleanly without breaks:
   

   $$
   \text{map} \longrightarrow \text{odom} \longrightarrow \text{base\_footprint} \longrightarrow \text{base\_link} \longrightarrow \text{velodyne}
   $$

3. **Map Inspection:** View the generated `.pgm` map and write a brief analysis comparing the SLAM result with the actual placement of obstacles in Gazebo.

## Lab Assessment & Submission Checklist

Submit a single `.zip` archive containing:

* \[ \] A screenshot showing RViz2 with the Go2 robot model, active `/scan` rays, and the generated `/map` in the `map` Fixed Frame.

* \[ \] Your generated `frames.pdf` showing the unified transform tree.

* \[ \] Your saved map files: `go2_sim_map.pgm` and `go2_sim_map.yaml`.

* \[ \] Written answers to the questions in Phase 1, Phase 2, and Phase 3.