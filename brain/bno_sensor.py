#!/usr/bin/env python3
import time
import json
import board
import os
import busio
import logging
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# -----------------------
# Logging Setup
# -----------------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# -----------------------
# BNO Sensor Loop
# -----------------------
def BNOFull(i2c_addr=0x4A,
            output_file="/home/equestria/brain/i2o/rohsen.json",
            target_hz=100.0,
            stop_event=None):
    """
    Liest kontinuierlich Daten vom BNO08X Sensor und schreibt sie atomar in JSON.

    Args:
        i2c_addr (int): I2C-Adresse des Sensors.
        output_file (str): Pfad zur JSON-Ausgabe.
        target_hz (float): Polling-Frequenz.
        stop_event (threading.Event, optional): Stop-Flag für sauberes Beenden.
    """
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c, address=i2c_addr)
    except Exception as e:
        logging.error(f"Fehler beim Initialisieren des BNO08X: {e}")
        return

    try:
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    except Exception as e:
        logging.error(f"Fehler beim Aktivieren der Sensor-Features: {e}")
        return

    time.sleep(0.2)

    period = 1.0 / target_hz
    next_ts = time.time()

    while True:
        if stop_event and stop_event.is_set():
            logging.info("BNO Sensor Loop beendet durch Stop-Event")
            break

        now = time.time()
        if now < next_ts:
            time.sleep(next_ts - now)
            continue
        next_ts += period
        ts = time.time()

        # -----------------------
        # Sensorwerte mit Fehlerbehandlung
        # -----------------------
        try:
            ax, ay, az = bno.acceleration
        except Exception:
            ax = ay = az = 0.0

        try:
            gx, gy, gz = bno.gyro
        except Exception:
            gx = gy = gz = 0.0

        try:
            mx, my, mz = bno.magnetic
        except Exception:
            mx = my = mz = 0.0

        try:
            qw, qx, qy, qz = bno.quaternion
        except Exception:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        data = {
            "timestamp": ts,
            "accel": {"raw_x": ax, "raw_y": ay, "raw_z": az},
            "gyro": {"raw_x": gx, "raw_y": gy, "raw_z": gz},
            "magnetometer": {"raw_x": mx, "raw_y": my, "raw_z": mz},
            "quaternion": {"w": qw, "x": qx, "y": qy, "z": qz},
        }

        # -----------------------
        # Atomar schreiben
        # -----------------------
        try:
            tmp = output_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, output_file)
        except Exception as e:
            logging.error(f"Fehler beim Schreiben von {output_file}: {e}")
