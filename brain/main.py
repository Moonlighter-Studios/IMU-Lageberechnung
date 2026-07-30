#!/usr/bin/env python3
import os
import sys
import threading
import time

# -----------------------
# Arbeitsverzeichnis setzen
# -----------------------
BASE_DIR = "/home/equestria/brain"
os.chdir(BASE_DIR)
sys.path.append(BASE_DIR)

# -----------------------
# Module importieren
# -----------------------
from bno_sensor import BNOFull
from imu_tracker import IMUTracker
from sichtfeld_marker import run_loop as combined_loop
from imu_visualizer import start_visualizer

if __name__ == "__main__":
    tracker = IMUTracker()

    INIT_SAMPLES = 100
    INIT_DELAY = 0.01
    for _ in range(INIT_SAMPLES):
        sensor = tracker._load_json(tracker.sensor_file)
        time.sleep(INIT_DELAY)

    t = threading.Thread(target=tracker.update_loop)
    t.start()

    threading.Thread(target=BNOFull, daemon=True).start()

    threading.Thread(target=combined_loop, daemon=True).start()

    start_visualizer()
