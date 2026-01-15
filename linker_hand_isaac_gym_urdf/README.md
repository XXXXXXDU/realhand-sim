# URDF Robot Joint Controller (Isaac Gym Version)

A URDF robot joint visualization and control tool based on PyQt5 and Isaac Gym. It supports real-time joint control and lets you observe the simulation results live.

## Features

- 🤖 Load URDF robot models
- 🎮 Real-time joint position control (slider UI)
- 📊 Joint state monitoring and display
- 🔄 Automatic synchronization of joint target values and current values
- 🎯 Joint limit checking and display
- 📱 Responsive GUI layout
- ⚡ GPU-accelerated physics simulation

## System Requirements

### Hardware Requirements
- NVIDIA GPU (CUDA-capable)
- At least 4GB VRAM
- Intel/AMD x64 processor

### Software Requirements
- Python 3.8
- CUDA Toolkit 11.x
- Ubuntu 20.04

## Installing Dependencies

### Create a Virtual Environment (Recommended)
```
conda create -n linker python=3.8
conda activate linker

# Install dependencies
pip install PyQt5 numpy urdfpy
```

### PyTorch (GPU Version)
Install PyTorch according to [these instructions](https://pytorch.org/get-started/locally/).

### Isaac Gym (Download Required from NVIDIA Developer Website)
Download Isaac Gym Preview 4 from the [official site](https://developer.nvidia.com/isaac-gym), then follow the installation instructions in the documentation.
```
cd isaacgym/python
pip install -e .
```

### Core Dependencies
```bash
# PyQt5 UI library
pip install PyQt5

# Numerical computing
pip install numpy

# URDF parsing
pip install urdfpy

```

## Usage

### Start the Program
```bash
python isaac_urdf.py
```

### Basic Workflow
1. **Load a URDF file**: Click the "Load URDF" button to select a robot URDF file
2. **Control joints**: Use the sliders on the left to adjust joint angles
3. **Monitor status**: The right panel shows current joint positions and simulation status
4. **Reset pose**: Click "Reset Positions" to restore the initial pose

### UI Overview
- **Left panel**: URDF loading and joint control sliders
- **Right panel**: Simulation status display and current joint position monitoring
- **Bottom status bar**: Shows current operation status and error messages

## Notes

⚠️ **Important reminders:**
1. Make sure the URDF file path does not contain Chinese characters
2. The first run of Isaac Gym may take a long time to initialize
3. If you run out of GPU memory, try reducing simulation complexity
4. Some complex URDF models may require tuning physics parameters

## FAQ

### Q: The program errors on startup: "Failed to create PhysX simulation"
A: Check that the NVIDIA driver and CUDA are installed correctly, and make sure your GPU supports CUDA.

### Q: The UI freezes after loading a URDF
A: The URDF file may be too complex—try simplifying the model or check the file format.

### Q: Joint control does not respond
A: Check whether the "Auto-update simulation" option is enabled.

## File Structure
```
.
├── isaac_urdf.py    # Main program file
├── README.md          # Documentation
└── [URDF directory]     # Stores robot URDF model files
```
