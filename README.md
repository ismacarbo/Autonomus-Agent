# 🚗 General Planner Simulator

## New: native C++ thesis simulator

The repository now includes a first native simulator in C++ for the thesis work:
- Dear ImGui + ImPlot UI
- 2D map with static obstacles
- differential-drive robot model aligned to the current Arduino/Raspberry robot
- simulated motor PWM, wheel speeds, and encoder ticks/delta ticks
- 2D lidar raycasting
- goal-directed navigation through Sabrina's `sel_jr` planner in gate-based mode

## New: real robot bridge in C++

The C++ side now also includes the first hardware bridge layer for moving from simulation to the real robot:
- POSIX serial wrapper for Linux
- `RPLidarA1` driver translated from the Python implementation
- full `RSP-v1` protocol encoder/decoder and stream parser
- `RSPSerialBridge` equivalent to the Python serial bridge for the Arduino/Raspberry link
- `RealRobotBridge` that unifies controller telemetry and LiDAR scans for the high-level agent
- `HardwarePlannerRunner` that reuses Sabrina's planner and maps the output to differential `PWM_L/PWM_R`

Main headers:
- `simulator/include/serial_port.h`
- `simulator/include/rsp_protocol.h`
- `simulator/include/rsp_serial_bridge.h`
- `simulator/include/rplidar_a1.h`
- `simulator/include/real_robot_bridge.h`

These modules are compiled together with the simulator target, so a normal build also validates the real-robot bridge code.

### Run the first real-robot loop

```bash
./build/simulator/thesis_robot_runner \
  --controller-port /dev/ttyACM0 \
  --lidar-port /dev/ttyUSB0
```

Current assumptions:
- known 2D map
- yaw and yaw-rate from IMU
- LiDAR-based local pose correction against the known map
- speed estimated from applied PWM until wheel encoders are added

### Build only the C++ simulator

```bash
cmake -S . -B build
cmake --build build --target thesis_planner_sim -j
```

### Run with GUI

```bash
./build/simulator/thesis_planner_sim
```

### Run headless for quick validation

```bash
./build/simulator/thesis_planner_sim --headless --max-steps 6000
```

The current demo scenario should terminate with `status=goal_reached`.

### Note on Python bindings

The legacy `generalEnv/bindings` path is no longer built by default.
If you want to work on it explicitly, enable it with:

```bash
cmake -S . -B build -DAUTONOMOUS_AGENT_BUILD_PYTHON_BINDINGS=ON
```

## Legacy Python playground

This part of the repository remains the older pygame-based environment.

This repository contains a pygame-based simulator playground with:
- **Lateral Frenet plant**
- **Longitudinal plant**
- **Double-track vehicle sandbox**
- **Combined mode** using Sabrina’s `sel_jr` (jerk + lateral command) through a `pybind11` module (**`gp_lat`**)

Optional: live logging to **Rerun Viewer**.

---

## ✅ Requirements

### System
- Linux (tested)
- CMake ≥ 3.14
- C++17 compiler (gcc/clang)
- Python ≥ 3.9 (tested with Python 3.13)

### Python packages
```bash
pip install pybind11 pygame numpy
```

### Git submodules
This project uses git submodules (Sabrina’s repo is under `third_party/`):

```bash
git clone --recurse-submodules <repo_url>
cd Autonomus-Agent
```

If you already cloned without submodules:
```bash
git submodule update --init --recursive
```

---

## 🧱 Build (CMake)

From repo root:
```bash
cmake -S . -B build
cmake --build build -j
```

Expected outputs:
- `build/generalEnv/bindings/gp_lat*.so`
- `generalEnv/gp_lat*.so` (copied next to Python entrypoint for easy import)

To verify which module you are importing:
```bash
python3 -c "import gp_lat; print(gp_lat.__file__)"
```

You should also see the Rerun helper functions if the module is the right one:
```bash
python3 -c "import gp_lat; print([x for x in dir(gp_lat) if 'rerun' in x])"
```

---

## ▶️ Run the simulator

```bash
cd generalEnv
python3 main.py
```

You’ll get a **mode selection menu**:
- `[1]` Lateral Frenet controller
- `[2]` Longitudinal controller
- `[3]` Double-track vehicle playground
- `[4]` Combined `sel_jr` (j+r together)

Controls are shown on-screen in each mode.

---

## 📊 Optional: Rerun logging

### Install Rerun Viewer (CLI in PATH)

Option A (cargo):
```bash
cargo binstall --force rerun-cli@0.27.3
```

Option B (pip):
```bash
pip install rerun-sdk==0.27.3
```

### How logging works
The C++ binding exposes:
- `gp_lat.rerun_spawn(app_id="gp_lat")`
- `gp_lat.rerun_connect(url="rerun+http://127.0.0.1:9876/proxy", app_id="gp_lat")`
- `gp_lat.set_cortex_rerun(True/False)` to enable/disable logging

In **Combined mode**:
- press **F1** to toggle cortex logging **ON/OFF**

Logged streams include:
- `cmd/j`, `cmd/r`
- `k0/x,v,a,n,b,c`
- `k1/x,v,a,n,b,c`

---

## 🧩 Notes on the C++ binding (`gp_lat`)

- Built with **pybind11**
- Links Sabrina’s C++ libs under `third_party/progettotesi`
- Rerun C++ SDK fetched via CMake `FetchContent`
- Rerun scalars are logged using `rerun::archetypes::Scalars(...)` (SDK 0.27.x compatible)
- Recording stream is initialized with `set_global()` + `set_thread_local()` to avoid thread-local warnings

---

## 🧹 Clean rebuild

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j
```

---

## 🆘 Troubleshooting

### pybind11 not found
```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=$(python3 -m pybind11 --cmakedir)
```

### “Failed to find Rerun Viewer executable in PATH”
Install the viewer (`rerun` must be available in your PATH), then rerun the simulator.

### “There is no currently active Recording stream available for the current thread”
The binding sets both global and thread-local streams.  
If you change the logging flow, ensure the recording stream is initialized and thread-local is set for the logging thread.
