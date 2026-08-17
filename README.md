# FAST-LIO2 in Gazebo — a reference LiDAR-inertial odometry sim

This repo is a small, **self-contained reference** that runs
[FAST-LIO2](https://github.com/hku-mars/FAST_LIO) (a tightly-coupled LiDAR +
IMU odometry / SLAM front-end) on a simulated robot in **Gazebo Classic**. A
3D LiDAR and an IMU in Gazebo feed FAST-LIO, which fuses them with an iterated
Error-State Kalman Filter (ESIKF) over an incremental **ikd-Tree** map and
publishes the robot's pose as `nav_msgs/Odometry` on `/odom` plus the TF
`odom -> base_link`. It is meant to be **studied and then re-deployed on an
RK3588 edge board** (ARM, CPU-only — *no NVIDIA/CUDA anywhere*). Correctness
and clarity are the goal, not features: the robot is a plain box, the world is
empty, and every moving part is documented below.

---

## 1. Data flow

```
  Gazebo (empty world) + urdf/robot_fastlio.urdf
  ┌───────────────────────────────────────────────────────────┐
  │   lidar3d  (ray sensor, 16 lines)      imu_sensor (200 Hz)  │
  │       │                                    │               │
  │  sensor_msgs/PointCloud2              sensor_msgs/Imu       │
  │  /lidar3d/points  (10 Hz)             /imu                  │
  │                                                             │
  │   planar_move  ── /cmd_vel in ──►  nav_msgs/Odometry        │
  │                                    /odom_gt  (GROUND TRUTH) │
  └───────┬─────────────────────────────────┬──────────────────┘
          │                                  │
          ▼                                  │
   sim/pc2_to_livox.py            (IMU passes straight through)
   PointCloud2 ─► livox CustomMsg                │
   (synthesises per-point offset_time)           │
          │                                       │
   livox_interfaces/CustomMsg                     │
   /livox/lidar_front                             │
          │                                       │
          ▼                                       ▼
   ┌───────────────────────────────────────────────────────┐
   │  fastlio2_ros2 :: ros2_localizer                       │
   │  recompiles fast_lio_core (ESIKF + ikd-Tree map),      │
   │  config = fast_lio_core/config/mid360_sim.yaml         │
   └───────────────────────┬───────────────────────────────┘
                            │
        nav_msgs/Odometry  /odom     +     TF  odom ─► base_link
```

`planar_move` is the "driver": it turns `/cmd_vel` into robot motion and also
publishes a **ground-truth** odometry on `/odom_gt`. It deliberately does **not**
publish a TF (`publish_odom_tf=false`) so that FAST-LIO is the single owner of
the `odom -> base_link` transform. Comparing `/odom` (FAST-LIO estimate)
against `/odom_gt` (truth) is how you judge the odometry quality.

---

## 2. Repo layout

```
fastlio_gazebo_sim/
├── README.md                     ← you are here
├── .gitignore
├── config/
│   └── fastdds_udp.xml           ← UDP-only DDS profile (SHM workaround)
├── fast_lio_core/                ← FAST-LIO-NON-ROS core (header-only + a few .cpp)
│   ├── include/                    ikd-Tree, IKFoM toolkit, common headers
│   ├── src/                        laserMapping.hpp, preprocess.cpp, fast_lio.hpp …
│   ├── config/
│   │   ├── mid360_sim.yaml        ← the config THIS sim uses
│   │   └── mid360.yaml            ← the real Livox Mid-360 config (for reference)
│   └── COLCON_IGNORE              ← so colcon does NOT build the core standalone
├── src/                          ← the two ROS 2 packages colcon builds
│   ├── livox_interfaces/          CustomMsg / CustomPoint message definitions only
│   └── fastlio2_ros2/             ROS 2 wrapper node `ros2_localizer`
├── sim/
│   └── pc2_to_livox.py           ← PointCloud2 → Livox CustomMsg bridge
├── urdf/
│   └── robot_fastlio.urdf        ← simplified, mesh-free robot + Gazebo sensors
├── launch/
│   └── fastlio_sim.launch.py     ← brings the whole pipeline up
└── rviz/
    └── fastlio.rviz              ← TF + /lidar3d/points + /odom
```

**Why `fast_lio_core` is not a package.** The wrapper does not link a
pre-built FAST-LIO library; it **recompiles the core straight from source**.
`src/fastlio2_ros2/CMakeLists.txt` points at the core with a *relative* path:

```cmake
set(FASTLIO_CORE "${CMAKE_CURRENT_SOURCE_DIR}/../../fast_lio_core")
add_executable(ros2_localizer
  src/ros2_localizer.cpp
  ${FASTLIO_CORE}/include/ikd-Tree/ikd_Tree.cpp
  ${FASTLIO_CORE}/src/preprocess.cpp)
```

From `src/fastlio2_ros2/`, `../../fast_lio_core` resolves to `<repo>/fast_lio_core`
— which is exactly this layout. The `COLCON_IGNORE` file keeps `colcon build`
focused on the two real ROS packages instead of trying to build the core's own
offline tool.

---

## 3. Prerequisites

- **Ubuntu 22.04 + ROS 2 Humble**
- `ros-humble-gazebo-ros-pkgs` (Gazebo Classic 11 + `gazebo_ros`)
- `ros-humble-robot-state-publisher`, `ros-humble-rviz2`,
  `ros-humble-teleop-twist-keyboard`
- **PCL ≥ 1.8**, **Eigen3**, **yaml-cpp**, **nlohmann-json (≥ 3.2)**, **OpenMP**
- A C++17 compiler
- **No CUDA. No GPU.** FAST-LIO here is pure CPU + OpenMP.

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
     ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
     libpcl-dev libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev
```

---

## 4. Build

> **The #1 gotcha (read this):** if you build from a shell that has **conda** or
> a **python venv** active, `colcon` picks up the wrong Python and the build
> fails with errors like `ModuleNotFoundError: No module named 'ament_package'`
> or empy/`em` import errors. **Deactivate conda/venv, strip them from `PATH`,
> and `unset PYTHONPATH` *before* you source ROS.**

```bash
# 1) Clean environment, THEN source ROS (order matters).
conda deactivate 2>/dev/null            # if you use conda
deactivate      2>/dev/null             # if a venv is active
unset PYTHONPATH
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'conda|/\.venv/' | paste -sd:)"
source /opt/ros/humble/setup.bash

# 2) Build (from the repo root).
cd <path>/fastlio_gazebo_sim
colcon build

# 3) Source the overlay so ros2 can find the new packages.
source install/setup.bash
```

This builds **two** packages: `livox_interfaces` (the CustomMsg types) and
`fastlio2_ros2` (the wrapper, which **recompiles the FAST-LIO core** — the first
build takes a minute or two because `laserMapping.hpp` is large). A couple of
harmless warnings scroll by (`pcap disabled`, a Boost `bind` placeholder
pragma); those are **not** errors.

*(Tip: a one-line `ros_env.sh` that does step 1 for you is a nice thing to keep
around — source it before every `colcon`/`ros2`/`launch` command.)*

---

## 5. Run

Headless (no GUI — just the solver and topics):

```bash
ros2 launch <path>/fastlio_gazebo_sim/launch/fastlio_sim.launch.py
```

With RViz:

```bash
ros2 launch <path>/fastlio_gazebo_sim/launch/fastlio_sim.launch.py rviz:=true
```

Drive it from a second terminal (source the same clean ROS env first):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then watch it work:

```bash
ros2 topic echo /odom --once      # FAST-LIO pose estimate
ros2 topic echo /odom_gt --once   # Gazebo ground truth (compare!)
ros2 run tf2_ros tf2_echo odom base_link
```

In RViz (fixed frame `odom`) you should see the `/lidar3d/points` cloud, the
red `/odom` arrow trail, the robot model, and the TF frames. As you teleop the
robot, `/odom` should closely track `/odom_gt`.

The launch file finds all repo paths **relative to itself** (or via
`$FASTLIO_SIM_REPO`), so there are no hardcoded absolute paths. It starts
`gzserver` only (headless); it spawns the robot after 4 s and starts the
bridge + localizer after 7 s so the sensors exist first.

---

## 6. The `pc2_to_livox.py` bridge — why it exists

FAST-LIO has two LiDAR front-ends. The **Livox path** (`lidar_type: 1`) does not
consume a generic `PointCloud2`; it wants a **`livox_interfaces/CustomMsg`**, in
which *every point carries its own `offset_time`* — the time (ns) between the
start of the sweep and when that particular point was measured. FAST-LIO uses
those per-point times to **de-skew** (motion-compensate) the scan against the
IMU-integrated trajectory.

Gazebo's ray sensor only emits a plain `PointCloud2` with no per-point timing.
The bridge fills that gap: for each point it computes the azimuth
`atan2(y, x)`, maps it across one 10 Hz sweep (0 … 100 ms) to synthesise an
`offset_time`, copies `x/y/z`, sets `reflectivity` from intensity, and packs it
into a `CustomMsg`. (Synthetic de-skew is approximate but fine at low speed.)

**Performance note:** the numpy point *reading* is vectorised, but building the
`CustomPoint[]` list is a **Python per-point loop — the throughput bottleneck**.
That is exactly why the URDF LiDAR is kept small: `180 horizontal × 16 vertical
≈ 2880 points/frame` at 10 Hz (~29k pts/s) is comfortable. Push the ray count
much higher and this loop, not FAST-LIO, becomes the limit. **On the real robot
the bridge is not used at all** — `livox_ros_driver2` publishes real `CustomMsg`
with true hardware `offset_time` directly.

---

## 7. Key parameters (`fast_lio_core/config/mid360_sim.yaml`)

| Key | Value | Meaning | Knob? |
|---|---|---|---|
| `common.lid_topic` | `/livox/lidar_front` | LiDAR input (matches bridge output) | wiring |
| `common.imu_topic` | `/imu` | IMU input | wiring |
| `common.time_sync_en` | `false` | soft LiDAR↔IMU time sync off (sim stamps agree) | rarely |
| `preprocess.lidar_type` | `1` | **1 = Livox** front-end (feeds on CustomMsg) | structural |
| `preprocess.scan_line` | `16` | number of LiDAR rings — **must match the sensor** | match-HW |
| `preprocess.blind` | `0.3` | drop returns closer than 0.3 m (self-hits) | **tune** |
| `mapping.acc_cov` | `0.1` | accelerometer measurement noise | **tune** |
| `mapping.gyr_cov` | `0.1` | gyroscope measurement noise | **tune** |
| `mapping.b_acc_cov` | `0.0001` | accel bias random-walk | rarely |
| `mapping.b_gyr_cov` | `0.0001` | gyro bias random-walk | rarely |
| `mapping.fov_degree` | `360` | sensor horizontal FOV | match-HW |
| `mapping.det_range` | `50.0` | max usable range (m) | **tune** |
| `mapping.extrinsic_est_en` | `false` | don't online-estimate LiDAR↔IMU extrinsic | rarely |
| `mapping.extrinsic_T` | `[0,0,0]` | LiDAR→IMU translation — **identity: both on `lidar_link`** | match-HW |
| `mapping.extrinsic_R` | identity | LiDAR→IMU rotation | match-HW |
| `publish.*` | — | scan/dense/body-frame output toggles | cosmetic |
| `pcd_save.pcd_save_en` | `false` | don't dump the map to `.pcd` | keep off |

**In sim the extrinsic is identity** because the URDF puts the LiDAR and the IMU
on the *same* `lidar_link`. On real hardware they are physically offset, so you
**must** set the true `extrinsic_T`/`extrinsic_R` (compare `mid360.yaml`, which
uses `extrinsic_T: [-0.011, -0.02329, 0.04412]`).

**Not in this YAML (baked into the core).** In this particular core the
downsample / map-size knobs are compile-time defaults in
`fast_lio_core/src/laserMapping.hpp` (around lines 331–334), not YAML fields:
`point_filter_num = 2` (keep every 2nd point), `filter_size_surf_min =
filter_size_map_min = 0.5` m (voxel leaf size), `cube_len = 200` m (side of the
local-map cube the ikd-Tree keeps around the robot). Those are the first things
to change when you tune for CPU / memory on the edge (see §8).

---

## 8. RK3588 / edge deployment notes

FAST-LIO here is **CPU + OpenMP only — there is no CUDA, cuDNN, or GPU code
path anywhere** in `fast_lio_core` or the wrapper. It cross-builds and runs
unchanged on aarch64. What to watch on an 8-core RK3588 (4× Cortex-A76 +
4× Cortex-A55, Mali GPU unused):

- **Thread count.** The wrapper hardcodes the OpenMP fan-out in
  `src/fastlio2_ros2/CMakeLists.txt`:
  ```cmake
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=3)
  ```
  For RK3588 set `MP_PROC_NUM` to **~4–6** (favour the big A76 cores while
  leaving headroom for the DDS/driver/nav stack) and rebuild. You can also cap
  at runtime without rebuilding: `export OMP_NUM_THREADS=4`. Don't set it to 8 —
  saturating every core starves the rest of the system and hurts latency.

- **Real sensor = Livox Mid-360 via `livox_ros_driver2`.** The driver publishes
  `livox_interfaces`-style `CustomMsg` **directly** with true per-point
  `offset_time`, so **drop `pc2_to_livox.py` entirely** and point
  `common.lid_topic` at the driver's topic (e.g. `/livox/lidar`). Use
  `mid360.yaml` as the starting config, with the **real** `extrinsic_T/R`.

- **Point rate & downsampling.** The Mid-360 emits **~200,000 points/s** — far
  more than this sim. Keep per-scan CPU bounded by raising `point_filter_num`
  (e.g. 3–4) and/or the voxel `filter_size_*` (e.g. 0.3–0.5 m), and by trimming
  `det_range`. (In this core those live in `laserMapping.hpp`; upstream FAST-LIO
  exposes them in YAML — either way they are your main throughput knobs.)

- **Memory.** The ikd-Tree map **grows with explored volume**. FAST-LIO trims
  points outside the `cube_len` box that moves with the robot, so tune `cube_len`
  down (e.g. 100 m) on an 8 GB board and watch RSS. Keep `pcd_save_en: false` —
  saving all frames to one `.pcd` can exhaust memory on long runs.

- **DDS transport.** Some ARM boards have flaky shared-memory DDS. If topics
  appear but never deliver, force UDP (see §9).

---

## 9. Known gotchas

1. **conda / `PYTHONPATH` break the build.** As in §4: deactivate conda/venv,
   `unset PYTHONPATH`, strip them from `PATH`, *then* `source /opt/ros/humble/setup.bash`.
   Symptom: `No module named 'ament_package'` / empy import errors during
   `colcon build`.

2. **Fast DDS shared-memory can be flaky** (WSL2, some RK3588 setups): topics
   list fine but carry no data. Force UDP-only with the bundled profile:
   ```bash
   export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds_udp.xml
   ```
   (or switch RMW, e.g. `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

3. **`planar_move` must NOT publish the odom TF.** In the URDF,
   `<publish_odom_tf>false</publish_odom_tf>` is mandatory: FAST-LIO owns the
   `odom -> base_link` transform. If planar_move also publishes it you get two
   writers fighting over the same TF edge and the pose in RViz jumps/tears.

4. **`use_sim_time`.** Everything runs on Gazebo's clock. The launch file sets
   `use_sim_time: true` on the nodes; if you run extra nodes by hand, set it
   there too or their timestamps won't line up.

5. **Start order.** The bridge and localizer must come up **after** the robot is
   spawned (its sensors have to exist first). The launch file already staggers
   this with `TimerAction`; if you start pieces manually, keep that order.

---

## 10. Approximating the real Mid-360 in sim (optional)

The default URDF LiDAR is a 16-line, ±15° puck for simplicity and light CPU.
The real Livox Mid-360 covers **360° × 59° (vertical −7°…+52°)**. To get closer
to it in sim, widen the `<vertical>` block of the `lidar3d` ray sensor in
`urdf/robot_fastlio.urdf`, e.g.

```xml
<vertical><samples>32</samples><resolution>1</resolution>
  <min_angle>-0.122</min_angle>   <!-- -7°  -->
  <max_angle>0.908</max_angle>    <!-- +52° -->
</vertical>
```

and set `preprocess.scan_line` to match. Remember the bridge's per-point Python
loop (§6) — every extra ray costs CPU there, so raise the count gradually.

---

### Credits / provenance

The FAST-LIO core is HKU-MARS' [FAST_LIO](https://github.com/hku-mars/FAST_LIO),
via the ROS-independent
[FAST-LIO-NON-ROS](https://github.com/BurhanMuhyiddin/FAST-LIO-NON-ROS) fork.
`livox_interfaces`, the `fastlio2_ros2` wrapper, the `pc2_to_livox.py` bridge,
and the sensor/plugin values in the URDF were extracted from a working sim in
`nav_ws_bbs`; the robot, world, launch, RViz, and this document were rebuilt
minimal and self-contained for study and edge deployment.
