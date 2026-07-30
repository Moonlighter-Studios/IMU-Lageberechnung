#!/usr/bin/env python3
import json
import numpy as np
import os
import random
os.environ["QT_QPA_PLATFORM"] = "xcb"
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# -----------------------
# CONFIG
# -----------------------

XYZ_FILE = "/home/equestria/brain/i2o/xyz.json"
SICHTFELD_FILE = "/home/equestria/brain/i2o/sichtfeld.json"
DOME_FILE = "/home/equestria/brain/i2o/dome.json"

paused = False
reinit = False

POLL_INTERVAL = 0.05
SICHTWEITE_D = 15

SHOW_GRID_X = True
SHOW_GRID_Y = True
SHOW_GRID_Z = False


FWD_LENGTH = 2.0
DOME_LENGTH = 2.0

DRONE_COLOR = "cyan"
FWD_COLOR = "red"
DOME_COLOR = "blue"
GROUND_POINT_COLOR = "green"
GROUND_POINT_ERROR_COLOR = "red"

# Optional: Skalierung, falls Sim-Werte nicht in Metern
SIM_SCALE = 1.0  # 1 Sim-Einheit = 1 Meter

# -----------------------
# FUNKTIONEN
# -----------------------

def on_key(event):
    global paused, reinit
    if event.key == 'p':
        paused = not paused
        print(f"Visualizer paused: {paused}")
    elif event.key == 'r':
        reinit = True
        print("Visualizer Reinit requested")
    elif event.key == 'm':

        pixel = [random.randint(0, 1920), random.randint(0, 1080)]
        write_json("ai.json", {"pixel": pixel})
        print(f"Pixel gesetzt: {pixel}")


def read_json(path, default=None):
    """JSON-Datei lesen, falls nicht vorhanden → default zurückgeben"""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return default

def quat_to_rot_matrix(q):
    """Quaternion [w,x,y,z] → 3x3 Rotationsmatrix"""
    r = R.from_quat([q[1], q[2], q[3], q[0]]) 
    return r.as_matrix()

def rotation_matrix_from_yaw_pitch(yaw_deg, pitch_deg):
    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)

    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)

    return np.array([
        [ cy*cp, -sy,  cy*sp],
        [ sy*cp,  cy,  sy*sp],
        [   -sp,   0,    cp ]
    ])
def write_json(file_name, data):
    path = os.path.join("/home/equestria/brain/o2i", file_name)
    tmp = path + ".tmp"
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(tmp, "w") as f:
        json.dump(data, f, separators=(",", ":"))
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)

def start_visualizer():
    global paused, reinit
    """Hauptloop: 3D-Plot Drohne + Sichtfeld"""
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', on_key)

    while True:
        if reinit:
            ax.cla()
            paused = False
            reinit = False
            print("Visualizer neu initialisiert")

        if paused:
            plt.pause(POLL_INTERVAL)
            continue

        # Drohnenposition + Rotation
        xyz = read_json(XYZ_FILE, {"x":0,"y":0,"z":0,"quaternion":{"w":1,"x":0,"y":0,"z":0}})
        X_D = xyz.get("x",0) * SIM_SCALE
        Y_D = xyz.get("y",0) * SIM_SCALE
        Z_D = xyz.get("z",0) * SIM_SCALE
        q = xyz.get("quaternion",{"w":1,"x":0,"y":0,"z":0})
        quat = [q.get("w",1), q.get("x",0), q.get("y",0), q.get("z",0)]
        R_d = quat_to_rot_matrix(quat)

        # ------------------ DOME-WINKEL ------------------
        dome_data = read_json(DOME_FILE, {"yaw_deg": 0.0, "pitch_deg": 0.0})
        R_dome = rotation_matrix_from_yaw_pitch(
            dome_data.get("yaw_deg", 0.0),
            dome_data.get("pitch_deg", 0.0)
        )

        # ------------------ GESAMT-ROTATION ------------------
        R_total = R_d @ R_dome

        ax.clear()

        # Limits um Drohne ±SICHTWEITE_D
        ax.set_xlim(X_D - SICHTWEITE_D, X_D + SICHTWEITE_D)
        ax.set_ylim(Y_D - SICHTWEITE_D, Y_D + SICHTWEITE_D)
        ax.set_zlim(max(0, Z_D - SICHTWEITE_D), Z_D + SICHTWEITE_D)

        # Grid nur X/Y alle 5 Meter
        ax.set_xticks(np.arange(X_D - SICHTWEITE_D, X_D + SICHTWEITE_D + 1, 5) if SHOW_GRID_X else [])
        ax.set_yticks(np.arange(Y_D - SICHTWEITE_D, Y_D + SICHTWEITE_D + 1, 5) if SHOW_GRID_Y else [])
        ax.set_zticks(np.arange(0, Z_D + SICHTWEITE_D + 1, 5) if SHOW_GRID_Z else [])

        # Grid nur einschalten, wenn mindestens eine Achse True
        ax.grid(SHOW_GRID_X or SHOW_GRID_Y or SHOW_GRID_Z)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # ----------------------
        # Drohne als Würfel zeichnen
        # ----------------------
        cube_size = 1.0  # halbe Kantenlänge
        cube_pts_local = np.array([
            [-cube_size, -cube_size, -cube_size],
            [cube_size, -cube_size, -cube_size],
            [cube_size, cube_size, -cube_size],
            [-cube_size, cube_size, -cube_size],
            [-cube_size, -cube_size, cube_size],
            [cube_size, -cube_size, cube_size],
            [cube_size, cube_size, cube_size],
            [-cube_size, cube_size, cube_size]
        ])

        cube_pts_world = (R_d @ cube_pts_local.T).T + np.array([X_D, Y_D, Z_D])

        # Kanten definieren
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7) 
        ]

        # Kanten plotten
        for i, j in edges:
            ax.plot([cube_pts_world[i, 0], cube_pts_world[j, 0]],
                    [cube_pts_world[i, 1], cube_pts_world[j, 1]],
                    [cube_pts_world[i, 2], cube_pts_world[j, 2]],
                    color=DRONE_COLOR)

        # Positions-Text über Würfelzentrum
        ax.text(X_D, Y_D, Z_D + cube_size + 0.2,
                f"X:{X_D:.2f}\nY:{Y_D:.2f}\nZ:{Z_D:.2f}",
                color="black", fontsize=10, ha='center')

        # ------------------ ORIENTIERUNGS-PFEILE ------------------

        origin = np.array([X_D, Y_D, Z_D])

        dir_drohne = R_d[:, 0] * FWD_LENGTH

        ax.quiver(
            origin[0], origin[1], origin[2],
            dir_drohne[0], dir_drohne[1], dir_drohne[2],
            color=FWD_COLOR, linewidth=2
        )

        dir_total = R_total[:, 0] * DOME_LENGTH

        ax.quiver(
            origin[0], origin[1], origin[2],
            dir_total[0], dir_total[1], dir_total[2],
            color=DOME_COLOR, linewidth=2
        )

        # Sichtfeldpunkte zeichnen
        sichtfeld = read_json(SICHTFELD_FILE)
        if sichtfeld and "bodenpunkt" in sichtfeld:
            xs, ys = [], []
            has_null = False
            for c in sichtfeld["bodenpunkt"]:
                if c["x"] is not None and c["y"] is not None:
                    x, y = c["x"], c["y"]
                    xs.append(x)
                    ys.append(y)
                    ax.scatter(x, y, 0, color=GROUND_POINT_COLOR, s=20)
                    ax.text(x, y, 0.2, f"X:{x:.2f}\nY:{y:.2f}", color="black", fontsize=8, ha='center')
                else:
                    has_null = True

            if xs:
                if has_null:
                    for x, y in zip(xs, ys):
                        ax.scatter(x, y, 0, color=GROUND_POINT_ERROR_COLOR, s=20)
                else:
                    xs.append(xs[0])
                    ys.append(ys[0])
                    ax.plot(xs, ys, [0]*len(xs), color=GROUND_POINT_COLOR)

        # ------------------ MARKER ------------------
        # Marker-History initialisieren, falls noch nicht vorhanden
        if not hasattr(start_visualizer, "_marker_history"):
            start_visualizer._marker_history = []

        # Aktuellen Marker aus marker.json laden
        marker_data = read_json("/home/equestria/brain/i2o/marker.json")
        if marker_data and "marker" in marker_data:
            m = marker_data["marker"]
            if m["x"] is not None and m["y"] is not None:
                if m not in start_visualizer._marker_history:
                    start_visualizer._marker_history.append(m)

        # Marker mit x=0 und y=0 löschen
        start_visualizer._marker_history = [
            marker for marker in start_visualizer._marker_history
            if not (marker["x"] == 0 and marker["y"] == 0)
        ]

        # Alle Marker aus der History zeichnen
        for marker in start_visualizer._marker_history:
            ax.scatter(marker["x"], marker["y"], 0, color="magenta", s=40)
            ax.text(marker["x"], marker["y"], 0.5,
                    f"Marker\nX:{marker['x']:.2f}\nY:{marker['y']:.2f}",
                    color="magenta", fontsize=10, ha='center')

        plt.draw()
        plt.pause(POLL_INTERVAL)
