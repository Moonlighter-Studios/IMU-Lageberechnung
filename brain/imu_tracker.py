#!/usr/bin/env python3
import json
import math
import os
import time
import threading
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# ----------------------- Konfiguration -----------------------
SENSOR_FILE = "/home/equestria/brain/i2o/rohsen.json"
OUTPUT_FILE = "/home/equestria/brain/i2o/xyz.json"

GRAVITY = 9.81
POLL_INTERVAL = 0.005

ZUPT_ACC_THRESH = 0.1
ZUPT_VEL_THRESH = 0.20
ZUPT_WINDOW_SIZE = 50

AUTO_CALIBRATE = True
CALIBRATION_DURATION = 10.0
CALIBRATION_INTERVAL = 0.01

ACC_BIAS = [0.0, 0.0, 0.0]
GYRO_BIAS = [0.0, 0.0, 0.0]

VELOCITY_ALPHA = 0.05


# ----------------------- IMU Tracker -----------------------
class IMUTracker:
    def __init__(self, stop_event=None):
        self.sensor_file = SENSOR_FILE
        self.output_file = OUTPUT_FILE
        self.stop_event = stop_event

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self._velocity_lp = [0.0, 0.0, 0.0]

        self.last_acc = [0.0, 0.0, 0.0]
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.last_time = time.monotonic()

        self.gravity = GRAVITY
        self.acc_bias = ACC_BIAS[:]
        self.gyro_bias = GYRO_BIAS[:]

        self.zupt_window = []

        if AUTO_CALIBRATE:
            self._auto_calibrate()

        self._write_json_atomic(self.output_file, {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        })

    # ----------------------- JSON Helper -----------------------
    def _load_json(self, path):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception:
            return None

    def _write_json_atomic(self, path, data):
        tmp = path + ".tmp"
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump(data, f, separators=(",", ":"))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)

    # ----------------------- Quaternion / Rotation -----------------------
    @staticmethod
    def normalize_quaternion(q):
        n = math.sqrt(sum(x*x for x in q))
        return [x/n for x in q]

    @staticmethod
    def quat_to_matrix(q):
        w,x,y,z = q
        return [
            [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x + y*y)]
        ]

    @staticmethod
    def rotate_vector(R,v):
        return [
            R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
        ]

    def _auto_calibrate(self):
        logging.info("Starte Auto-Kalibrierung (10s)")
        acc_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        samples = 0
        start_time = time.time()
        while time.time() - start_time < 10.0:
            sensor = self._load_json(SENSOR_FILE)
            if sensor:
                for i, k in enumerate(["raw_x", "raw_y", "raw_z"]):
                    acc_sum[i] += sensor.get("accel", {}).get(k, 0.0)
                    gyro_sum[i] += sensor.get("gyro", {}).get(k, 0.0)
                samples += 1
            time.sleep(0.01)
        if samples > 0:
            self.acc_bias = [x / samples for x in acc_sum]
            self.gyro_bias = [x / samples for x in gyro_sum]
        logging.info(f"Auto-Kalibrierung abgeschlossen: Acc-Bias={self.acc_bias}, Gyro-Bias={self.gyro_bias}")

    # ----------------------- Update Loop -----------------------
    def update(self):
        sensor = self._load_json(self.sensor_file)
        if not sensor:
            return

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            return

        ax = sensor.get("accel", {}).get("raw_x",0.0) - self.acc_bias[0]
        ay = sensor.get("accel", {}).get("raw_y",0.0) - self.acc_bias[1]
        az = sensor.get("accel", {}).get("raw_z",0.0) - self.acc_bias[2]

        # ----------------------- Quaternion direkt vom Sensor -----------------------
        q = [
            sensor.get("quaternion", {}).get("w",1.0),
            sensor.get("quaternion", {}).get("x",0.0),
            sensor.get("quaternion", {}).get("y",0.0),
            sensor.get("quaternion", {}).get("z",0.0)
        ]
        self.q = self.normalize_quaternion(q)
        Rm = self.quat_to_matrix(self.q)

        acc_world = self.rotate_vector(Rm, [ax, ay, az])
        lin_acc = [acc_world[0], acc_world[1], acc_world[2]-self.gravity]

        # ----------------------- Velocity / Position Integration -----------------------
        for i in range(3):
            self.velocity[i] += 0.5*(lin_acc[i]+self.last_acc[i])*dt
            self._velocity_lp[i] = VELOCITY_ALPHA*self.velocity[i] + (1-VELOCITY_ALPHA)*self._velocity_lp[i]
            self.velocity[i] = self._velocity_lp[i]
            self.position[i] += self.velocity[i]*dt

        self.last_acc = lin_acc[:]

        # ---------------- Adaptive ZUPT ----------------
        self.zupt_window.append((lin_acc[:], self.velocity[:]))
        if len(self.zupt_window) > ZUPT_WINDOW_SIZE:
            self.zupt_window.pop(0)

        acc_norms = [math.sqrt(a[0]**2 + a[1]**2 + a[2]**2) for a,_ in self.zupt_window]
        vel_norms = [math.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for _,v in self.zupt_window]

        acc_mean = sum(acc_norms)/len(acc_norms)
        acc_std = (sum((x-acc_mean)**2 for x in acc_norms)/len(acc_norms))**0.5
        vel_mean = sum(vel_norms)/len(vel_norms)
        vel_std = (sum((x-vel_mean)**2 for x in vel_norms)/len(vel_norms))**0.5

        # ZUPT nur auf x,y optional, z wird nicht auf 0 gesetzt
        if acc_mean + 1.5*acc_std < ZUPT_ACC_THRESH and vel_mean + 1.5*vel_std < ZUPT_VEL_THRESH:
            self.velocity[0] = 0.0
            self.velocity[1] = 0.0
            self._velocity_lp[0] = 0.0
            self._velocity_lp[1] = 0.0
            # z Velocity bleibt, damit Auf/Ab-Bewegung möglich ist

        # ---------------- JSON Ausgabe ----------------
        self._write_json_atomic(self.output_file, {
            "x": 0,  self.position[0],
            "y": 0, self.position[1],
             "z": 0, max(0.0, self.position[2]),
            "vx": self.velocity[0],
            "vy": self.velocity[1],
            "vz": self.velocity[2],
            "quaternion": {"w": self.q[0], "x": self.q[1], "y": self.q[2], "z": self.q[3]}
        })

    def update_loop(self):
        while not (self.stop_event and self.stop_event.is_set()):
            self.update()
            time.sleep(POLL_INTERVAL)


# ----------------------- Main -----------------------
if __name__ == "__main__":
    stop_event = threading.Event()
    tracker = IMUTracker(stop_event)
    try:
        tracker.update_loop()
    except KeyboardInterrupt:
        stop_event.set()
