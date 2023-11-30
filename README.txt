# MotionNav

MotionNav is an innovative project that aims to redefine the driving experience 
through the integration of gesture control technology. Utilizing the powerful 
capabilities of Carla and eCAL, MotionNav introduces a new dimension to vehicle interaction,
allowing users to seamlessly navigate and control various features with intuitive gestures.

## Table of Contents

- [Features]
- [Installation]

## Features

Gesture-Based Navigation: 
    Say goodbye to traditional controls. MotionNav empowers users to navigate and
    interact with their vehicles through natural, gesture-based commands.

Carla Integration:
    Leveraging the capabilities of Carla, MotionNav ensures a robust and 
    realistic driving environment, enhancing the overall user experience.

eCAL Integration: 
    The project seamlessly integrates with eCAL, providing efficient and
    reliable communication between vehicle components, sensors, and the gesture control system.

## Installation

### Prerequisites

- Python (3.7.16)
- Anaconda (1.12.1)
- Python Packages:
    - carla             0.9.15
    - matplotlib        3.5.3
    - pygame            2.5.2
    - opencv-python     4.6.1.78
    - numpy             1.21.6
    - open3d            0.17.0

### Steps

1. **Download and Install Anaconda:**
   - Visit [Anaconda Downloads](https://www.anaconda.com/products/distribution)
   - Choose the appropriate installer for your operating system.
   - Follow the installation instructions.

2. **Create a Anaconda Enviroment and installing Python Packages:**
    1. Start Anaconda Prompt
    2. create --name [Enviroment_Name] python=[Python_Version]
    3. activate [Enviroment_Name]
    4. pip install [Package_Name]

3. **Download and Install Carla:**
    - Visit [Carla Documentation](https://carla.readthedocs.io/en/latest/start_quickstart/)
    - Follow the installation instructions.

4. **Download and Install eCAL:**
    - Visit [eCAL Website](https://eclipse-ecal.github.io/ecal/index.html)
    - Follow the installation instructions.

5. **Run the Project:**
    - Start the Carla.exe
    - Start the Simulation.py Script
    | The Port '2000' need to be used | 
