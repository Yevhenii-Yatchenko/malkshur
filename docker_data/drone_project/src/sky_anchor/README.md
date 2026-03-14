**Sky Anchor**
=============================

This project provides a drone drift correction system that works both as a 
simulation GUI and a real drone application using a USB camera. 
The system detects movement drift using ORB feature matching and sends 
correction commands to a flight controller. (pymavlink and pixhawk 6c)

------------------------------------------------------------
**INSTALLATION**
------------------------------------------------------------

**1. Create a Conda Environment or Python Environment**
   ---------------------------
   This project uses Python 3.8 in a Conda environment.

   If env already exist:

   ```conda remove -n sky_anchor --all```

   Run the following command:

   ```conda create -n sky_anchor python=3.8```

   ```conda activate sky_anchor```

**2. Install Dependencies**
   ----------------------
   Run:

   ```pip install pillow```

   ```pip install python-dotenv```
   
   ```pip install pymavlink```

   ```pip install pyserial```


**2.1. Install OpenCV**
   ----------------------
   **All commands must be run in conda environment**

   **Windows:**
1. Download wheel from official github repo: https://github.com/cudawarped/opencv-python-cuda-wheels/releases

2. Install wheel (for example)


   ```pip install opencv_contrib_python_rolling-4.12.0.86-cp37-abi3-win_amd64.whl```


3. Verify installation (must return something like: Version: 4.9.0 CUDA devices: 1)


   ```python -c "import cv2; print(cv2.__version__); print(cv2.cuda.getCudaEnabledDeviceCount())"```


   **Jetson:**

1. Install opencv from wheel


   ```pip install opencv_python-4.5.0-py3-none-any.whl```


**3. Configure the Environment**
   ---------------------------
   Create a ".env" file in the project root (or rename .env.example to .env).

   ```cp .env.example .env```
   

------------------------------------------------------------
**USAGE**
------------------------------------------------------------

**1. Run the Simulation GUI (for testing)**
   -------------------------------------
   Run:

   python test_gui.py

   - Load an Image: Click "Load Image" and select a test image.
   - Define a Reference Area: Click and drag to draw a blue box on the image.
   - Move/Rotate Manually: Use movement & rotation buttons to simulate drift.
   - Run Performance Test: Applies random moves for 1 minute in the background.
   - Run Visual Test: Runs for 1 minute while drawing moves on the GUI.

**2. Run on a Real Drone (with a USB Camera)**
   ----------------------------------------
   Run:

   python main.py

   This script will:

   - Open the USB camera and capture a reference frame.
   - Continuously capture new frames and detect drift (dx, dy, angle).
   - Send correction commands to the flight controller if needed.
   - If feature matching fails too many times, a new reference frame is captured.

------------------------------------------------------------
**NOTES**
------------------------------------------------------------

- For debugging, set "DRONE_DEBUG=True" in ".env" to log detailed shift estimates.
- For best performance, ensure the USB camera is stable and has a clear, textured
  view to track movement accurately.
