# 🚗 General Planner Simulator (Python + C++ bindings)

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