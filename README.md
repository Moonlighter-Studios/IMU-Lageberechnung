# IMU-Lageberechnung
# Drone Ground Projection

A modular Python framework for determining the orientation of a drone and projecting the camera field of view onto the ground using IMU sensor fusion, quaternion mathematics and camera geometry.

The project was developed as part of a technical seminar project. Its primary objective is the calculation of drone orientation, camera viewing direction and the projection of AI-detected image coordinates onto real-world ground coordinates.

---

## Features

* BNO085 IMU data acquisition
* Quaternion-based orientation estimation
* Dead Reckoning position estimation
* Camera field-of-view projection
* AI marker projection onto the ground
* Independent dome rotation support
* Real-time 3D visualization
* JSON-based communication between modules
* Automatic logging system

---

## Software Architecture

BNO085
   │
   ▼
BNO_sensor.py
   │
   ▼
IMU-Tracker.py
   │
   ├───────────────┐
   ▼               │
 xyz.json          │
 Quaternion        │
   │               │
   ▼               │
Sichtfeld_Marker.py ◄──────────────┐
   ▲                               │
   │                               │
 ai.json     dome.json     fov.json
   │
   ▼
marker.json
sichtfeld.json
   │
   ▼
Visualizer.py

---

## Hardware

* Raspberry Pi 5
* BNO085 IMU
* Pan/Tilt Camera (Dome)

---

## Software Requirements

* Python 3.13
* SciPy
* NumPy
* Matplotlib
* Adafruit CircuitPython BNO08X

---

## Installation

Clone the repository

Install all dependencies

pip install -r requirements.txt

Run the application

python main.py

---

## Data Flow

The modules communicate using JSON files.

rohsen.json
        │
        ▼
IMU-Tracker.py
        │
        ▼
xyz.json

ai.json
dome.json
fov.json
xyz.json
        │
        ▼
Sichtfeld_Marker.py
        │
        ├── marker.json
        └── sichtfeld.json

marker.json
sichtfeld.json
xyz.json
        │
        ▼
Visualizer.py

---

## Mathematical Concepts

The project implements the following mathematical methods:

* Sensor Fusion
* Dead Reckoning
* Quaternion Rotation
* Rotation Matrices
* Coordinate Transformations
* Camera Projection
* Inverse Perspective Projection
* Ray–Plane Intersection

---

## Dependencies

This project relies on several open-source libraries:

* Adafruit CircuitPython BNO08X
* SciPy
* NumPy
* Matplotlib

Special thanks to the maintainers of these libraries.

---

## Current Limitations

* Position estimation is based on Dead Reckoning and therefore accumulates drift over time.
* A flat ground plane is assumed.
* No GNSS or external position reference is currently used.

---

## Posible Improvements

* GNSS integration
* Terrain model support
* Barometric altitude estimation
* Improved drift compensation
* Additional visualization features

---

## License

See the LICENSE file for details.

---

## Author

Leon Kahric

Developed as part of a technical seminar project on drone orientation, camera projection and ground coordinate calculation.

