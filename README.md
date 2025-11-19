# 🚗 General Planner Simulator - Build & Execution Instructions

## 1️⃣ Requirements
- Python ≥ 3.9  
- CMake ≥ 3.14  
- pybind11 installed (for C++ binding)  
- pygame and numpy for visualization and numerical routines  

Install everything on Linux with:

    sudo apt install python3-dev python3-pip cmake build-essential
    pip install pybind11 pygame numpy

---

## 2️⃣ Clone the Project

    git clone --recurse-submodules https://github.com/<your_repo>/autonomous-agent.git
    cd autonomous-agent

If you already cloned without submodules:

    git submodule update --init --recursive

---

## 3️⃣ Build

Build the Python module `gp_lat` (which links to Sabrina’s C++ function `r(...)`) and prepare the CMake targets:

    cmake -S . -B build
    cmake --build build -j

This will generate:
- the Python library `gp_lat.so` inside `generalEnv/bindings`
- the intermediate binaries required for the simulator

---

## 4️⃣ Run the Simulator

You can launch directly from **CMake**:

    cmake --build build --target run_sim

Or manually from Python:

    cd generalEnv
    python3 main_lateral.py

---

## 5️⃣ Main Dependencies
- `pybind11` — C++/Python binding layer  
- `pygame` — 2D rendering  
- `numpy` — numerical integration and math utilities

---

## 6️⃣ Tips

To clean and rebuild from scratch:

    rm -rf build
    cmake -S . -B build
    cmake --build build -j

On Windows, use:

    cmake --build build --config Release

---

## 7️⃣ If You Get “pybind11 Not Found”

Add the pybind11 location manually to CMake:

    cmake -S . -B build -DCMAKE_PREFIX_PATH=$(python3 -m pybind11 --cmakedir)

---

## ✅ Summary

1. Install dependencies  
2. Clone the repo with submodules  
3. Build using `cmake`  
4. Run with `run_sim` or `python3 main_lateral.py`

Everything else (planner integration, renderer, bindings) is handled automatically.

---

