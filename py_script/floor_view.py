import socket
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy as np
from scipy.optimize import least_squares
import json
import os

# ---------------- CONFIGURATION ---------------- #
HOST = "0.0.0.0"
PORT = 7007

ROOM_WIDTH = 300
ROOM_HEIGHT = 400
IMAGE_FILE = "floorplan.png"

# لازم يكون ترتيبها مطابق لـ ANCHOR_IDS
ANCHOR_IDS = [1, 2, 3, 4]

ANCHOR_POSITIONS = np.array([
    [15, 5],     # A1
    [290, 5],    # A2
    [165, 625],  # A3
    [450, 620],  # A4  <-- عدّلها حسب قياساتك
], dtype=float)

MIN_DISTANCE_CM = 5
MAX_DISTANCE_CM = 2000
# ------------------------------------------------ #

latest_distances = None   # list length 4
latest_rssi = None        # list length 4
data_lock = threading.Lock()
server_running = True
buffer = ""

def valid_distance(d):
    return (d is not None) and (MIN_DISTANCE_CM <= d <= MAX_DISTANCE_CM)

def trilaterate(distances, anchors_xy):
    distances = np.array(distances, dtype=float)
    anchors = np.array(anchors_xy, dtype=float)

    def residuals(p):
        x, y = p
        return np.hypot(x - anchors[:, 0], y - anchors[:, 1]) - distances

    initial_guess = np.mean(anchors, axis=0)
    try:
        res = least_squares(residuals, initial_guess)
        return res.x if res.success else None
    except Exception as e:
        print("Trilateration error:", e)
        return None

def wifi_server():
    global latest_distances, latest_rssi, server_running, buffer

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        s.settimeout(1.0)
        print("Server listening on {}:{}".format(HOST, PORT))

        while server_running:
            try:
                conn, addr = s.accept()
            except socket.timeout:
                continue

            print("Connected by", addr)
            with conn:
                conn.settimeout(1.0)
                while server_running:
                    try:
                        data = conn.recv(2048)
                        if not data:
                            break
                        buffer += data.decode("utf-8", errors="ignore")

                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue

                            try:
                                j = json.loads(line)
                                anchors = j["anchors"]  # dict

                                d_list = []
                                r_list = []

                                # نقرأ حسب ANCHOR_IDS وبنفس الترتيب
                                for aid in ANCHOR_IDS:
                                    key = "A{}".format(aid)
                                    aobj = anchors[key]
                                    d = float(aobj["distance"])   # filtered_distance عندك
                                    r = float(aobj.get("rssi", np.nan))
                                    d_list.append(d)
                                    r_list.append(r)

                                # فلترة المسافات
                                if not all(valid_distance(d) for d in d_list):
                                    print("Invalid distances:", d_list)
                                    continue

                                with data_lock:
                                    latest_distances = d_list
                                    latest_rssi = r_list

                            except Exception as e:
                                print("Bad line:", line[:120], " err=", e)

                    except socket.timeout:
                        continue
                    except ConnectionResetError:
                        break

            print("Client disconnected")

# ---------------- PLOTTING ---------------- #
fig, ax = plt.subplots(figsize=(10, 8))

if os.path.exists(IMAGE_FILE):
    bg = mpimg.imread(IMAGE_FILE)
    ax.imshow(bg, extent=[0, ROOM_WIDTH, 0, ROOM_HEIGHT], origin="lower", alpha=0.6, zorder=-1)

anchor_texts = []
for i, (x, y) in enumerate(ANCHOR_POSITIONS):
    ax.plot(x, y, "^", markersize=12, linestyle="None", label="A{}".format(ANCHOR_IDS[i]))
    anchor_texts.append(ax.text(x, y + 20, "", ha="center", va="bottom"))

(tag_dot,) = ax.plot([], [], "o", markersize=10, label="Tag")
(path_line,) = ax.plot([], [], "-", alpha=0.5, linewidth=1, label="Path")
path_x, path_y = [], []

ax.set_xlim(0, ROOM_WIDTH)
ax.set_ylim(0, ROOM_HEIGHT)
ax.set_aspect("equal")
ax.grid(True, linestyle="--", alpha=0.7)
ax.legend(loc="upper right")
ax.set_title("UWB Tag Tracking (4 Anchors)")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")

def update(_):
    global path_x, path_y

    with data_lock:
        d_list = latest_distances
        r_list = latest_rssi

    if not (d_list and r_list):
        return (tag_dot, path_line) + tuple(anchor_texts)

    # تحديث RSSI فوق الأنكور
    for txt, sig in zip(anchor_texts, r_list):
        if np.isfinite(sig):
            txt.set_text("{:.1f} dBm".format(sig))
        else:
            txt.set_text("")

    pos = trilaterate(d_list, ANCHOR_POSITIONS)
    if pos is not None:
        x_cm, y_cm = pos
        if 0 <= x_cm <= ROOM_WIDTH and 0 <= y_cm <= ROOM_HEIGHT:
            tag_dot.set_data([x_cm], [y_cm])
            path_x.append(float(x_cm))
            path_y.append(float(y_cm))
            if len(path_x) > 150:
                path_x.pop(0); path_y.pop(0)
            path_line.set_data(path_x, path_y)

    return (tag_dot, path_line) + tuple(anchor_texts)

if __name__ == "__main__":
    t = threading.Thread(target=wifi_server, daemon=True)
    t.start()

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False, blit=False)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        server_running = False
        plt.close()
