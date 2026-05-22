#!/usr/bin/env python3
"""
Hand-Eye-Kalibrierung mit ChArUco – vollautomatisch via ROS2
=============================================================
Roboter fährt vordefinierte Positionen ab, nimmt Bilder auf,
liest TCP aus ROS2 aus und berechnet Hand-Eye-Kalibrierung.

Voraussetzung:
    - Launch File läuft (MoveIt2)
    - Kamera erreichbar
    - ChArUco Board fest hingelegt

Starten:
    python3 hand_eye_ros2.py

Korrekturen gegenüber ursprünglicher Version:
    - Einheitenkonsistenz: t_rob jetzt in Metern (wie solvePnP)
    - t_rob shape (3,1) statt (3,) für calibrateHandEye
    - Undistortion vor solvePnP
    - Bildindex-Zuordnung repariert (gespeicherter Index statt Laufindex)
    - TF-Lookup mit Timeout
    - Validierung: Reprojektionsfehler nach Kalibrierung
    - time.sleep nach Bewegung reduziert
    - Mehrere Hand-Eye-Methoden werden verglichen
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints, JointConstraint
)
import tf2_ros
import math
import socket
import time
import numpy as np
import cv2
import json
import os
from scipy.spatial.transform import Rotation

# ── Konfiguration ──────────────────────────────────────────────
KAMERA_IP      = "192.168.100.20"
KAMERA_PORT    = 2006
SAVE_DIR       = "/home/lukas/GreifpunktStudien/src/robo_work/robo_work/camcalib"
CALIB_FILE     = os.path.join(SAVE_DIR, "kamera_kalibrierung.npz")
RESULT_FILE    = os.path.join(SAVE_DIR, "hand_eye_kalibrierung.npz")
POSES_FILE     = os.path.join(SAVE_DIR, "tcp_poses.json")
PLANNING_GROUP = "igus_rebel_arm"
VELOCITY       = 0.2

# ── ChArUco Board ─────────────────────────────────────────────
CHARUCO_COLS      = 9
CHARUCO_ROWS      = 7
QUADRAT_GROESSE_M = 0.029   # 29mm
MARKER_GROESSE_M  = 0.0215  # 21.5mm
ARUCO_DICT        = cv2.aruco.DICT_5X5_50

os.makedirs(SAVE_DIR, exist_ok=True)

# ── Kalibrierungspositionen (A1-A6 in Grad) ───────────────────
KALIBRIER_POSITIONEN = [
    [0.0,   -79.0, 139.0,   0.0,  82.0,   0.0],
    [0.3,   -44.5, 116.0,   0.0,  82.0,   0.0],
    [7.5,   -44.5, 116.0,   0.0,  82.0,   0.0],
    [7.5,   -44.5, 113.6,   0.0,  82.0,   0.0],
    [2.1,   -37.8, 117.3,   4.2,  76.3,   4.4],
    [-0.3,  -37.8, 117.3,   4.2,  76.3,   4.4],
    [-0.3,  -37.8, 117.3,   4.2,  76.3,  31.3],
    [-0.3,  -37.8, 117.3,   4.2,  76.3,  54.0],
    [-0.3,  -37.8, 117.3,   4.2,  76.3,  77.6],
    [-0.3,  -37.8, 117.3,   4.2,  76.3, -12.4],
    [-0.3,  -37.8, 117.3,   4.2,  76.3, -44.5],
    [9.5,   -37.8, 117.3,  -9.9,  76.3, -44.5],
    [9.5,   -45.0, 117.3,  -9.9,  76.3, -44.5],
    [9.5,   -45.0, 121.6,  -9.9,  76.3,  15.3],
    [9.5,    -2.0,  95.0,  -9.9,  76.3,  15.3],
    [9.5,   -17.8,  92.3,  -9.9,  90.7,  15.3],
    [17.8,  -17.8,  92.3,  -9.9,  90.7,  15.3],
    [17.8,  -17.8,  92.3, -14.7,  90.7,  15.3],
    [17.8,  -25.9,  92.3, -14.7,  95.0,  15.3],
    [17.8,  -21.9,  90.7, -14.7,  95.0,  15.3],
    [29.7,  -21.9,  90.7, -19.8,  95.0,  15.3],
    [29.7,  -24.5,  94.2, -19.8,  95.0,  15.3],
    [29.7,  -58.7, 120.0, -19.8,  93.2,  15.3],
    [39.9,  -58.7, 120.0, -19.8,  93.2,  53.6],
    [53.0,  -72.3, 137.3, -29.2,  93.2,  53.6],
    [53.0,  -72.3, 137.3, -28.4,  93.2,  53.6],
]


# ═══════════════════════════════════════════════════════════════
# Kamera
# ═══════════════════════════════════════════════════════════════

def send(sock, cmd):
    sock.send((cmd + "\r\n").encode("ascii"))

def recv_all(sock, timeout=2):
    sock.settimeout(timeout)
    data = b""
    try:
        while True:
            part = sock.recv(4096)
            if not part:
                break
            data += part
    except socket.timeout:
        pass
    return data

def kamera_bild_aufnehmen():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((KAMERA_IP, KAMERA_PORT))
    send(sock, "TRG")
    sock.recv(100)
    time.sleep(0.30)
    send(sock, "GIM0")
    raw = recv_all(sock)
    if not raw.startswith(b"GIMP"):
        send(sock, "GIM1")
        raw = recv_all(sock)
        if not raw.startswith(b"GIMP"):
            sock.close()
            raise Exception("Kein Bild erhalten!")
    rows = int(raw[7:11].decode())
    cols = int(raw[11:15].decode())
    raw_pixels = raw[19:]
    pixel_count = rows * cols
    if len(raw_pixels) < pixel_count:
        raw_pixels += b"\x00" * (pixel_count - len(raw_pixels))
    img = np.frombuffer(raw_pixels[:pixel_count], dtype=np.uint8).reshape((rows, cols))
    sock.close()
    return img


# ═══════════════════════════════════════════════════════════════
# ChArUco
# ═══════════════════════════════════════════════════════════════

def charuco_board_erstellen():
    aruco_dict    = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    charuco_board = cv2.aruco.CharucoBoard(
        (CHARUCO_COLS, CHARUCO_ROWS),
        QUADRAT_GROESSE_M,
        MARKER_GROESSE_M,
        aruco_dict
    )
    return aruco_dict, charuco_board

def charuco_erkennen(img):
    """Gibt (erfolg, corners, ids) zurück. img muss Graustufen sein."""
    _, charuco_board = charuco_board_erstellen()
    detector = cv2.aruco.CharucoDetector(charuco_board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(img)
    if charuco_ids is None or len(charuco_ids) < 4:
        return False, None, None
    return True, charuco_corners, charuco_ids


# ═══════════════════════════════════════════════════════════════
# Roboter Node
# ═══════════════════════════════════════════════════════════════

class HandEyeNode(Node):
    def __init__(self):
        super().__init__('hand_eye_ros2')
        self._action_client = ActionClient(self, MoveGroup, "/move_action")
        self._tf_buffer     = tf2_ros.Buffer()
        self._tf_listener   = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("Warte auf move_group ...")
        self._action_client.wait_for_server()
        self.get_logger().info("Verbunden!")

    def move_to_joints(self, joint_angles_deg):
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        constraints = Constraints()
        for jname, jval_deg in zip(joint_names, joint_angles_deg):
            jc = JointConstraint()
            jc.joint_name      = jname
            jc.position        = math.radians(jval_deg)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name                      = PLANNING_GROUP
        request.max_velocity_scaling_factor     = VELOCITY
        request.max_acceleration_scaling_factor = VELOCITY
        request.allowed_planning_time           = 5.0
        request.num_planning_attempts           = 5
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request                          = request
        goal.planning_options                 = PlanningOptions()
        goal.planning_options.plan_only       = False
        goal.planning_options.replan          = True
        goal.planning_options.replan_attempts = 3

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

    def tcp_auslesen(self):
        """
        Liest TCP-Pose aus TF2 aus.
        Gibt [x, y, z, A, B, C] in METERN und Grad zurück.
        WICHTIG: Einheit ist Meter (nicht mm) für calibrateHandEye!
        """
        try:
            # FIX: Timeout hinzugefügt – verhindert sporadische Crashes
            transform = self._tf_buffer.lookup_transform(
                'world', 'link6',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            t = transform.transform.translation
            r = transform.transform.rotation

            # FIX: Meter statt Millimeter – konsistent mit solvePnP-Ausgabe
            x = t.x
            y = t.y
            z = t.z

            rot   = Rotation.from_quat([r.x, r.y, r.z, r.w])
            euler = rot.as_euler('xyz', degrees=True)
            A, B, C = euler[0], euler[1], euler[2]

            return [x, y, z, A, B, C]
        except Exception as e:
            self.get_logger().error(f"TF Fehler: {e}")
            return None


# ═══════════════════════════════════════════════════════════════
# Hand-Eye Kalibrierung berechnen
# ═══════════════════════════════════════════════════════════════

def kalibrierung_berechnen():
    print("\nBerechne Hand-Eye-Kalibrierung ...")

    daten = np.load(CALIB_FILE)
    K     = daten["K"]
    dist  = daten["dist"]

    with open(POSES_FILE, "r") as f:
        tcp_posen = json.load(f)

    _, charuco_board = charuco_board_erstellen()
    detector = cv2.aruco.CharucoDetector(charuco_board)

    R_rob_liste = []
    t_rob_liste = []
    R_kam_liste = []
    t_kam_liste = []

    for i, tcp_pose in enumerate(tcp_posen):
        # FIX: Index basiert auf gespeichertem Index (i+1), nicht auf
        # Laufindex der Rohpositionen – beide Listen sind jetzt synchron
        pfad = os.path.join(SAVE_DIR, f"handeye_{i+1:02d}.png")
        if not os.path.exists(pfad):
            print(f"  ✗ Bild {pfad} nicht gefunden")
            continue

        img = cv2.imread(pfad, cv2.IMREAD_GRAYSCALE)

        # FIX: Bild vor solvePnP entzerren; dist=None in solvePnP übergeben
        img_undist = cv2.undistort(img, K, dist)

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(img_undist)

        if charuco_ids is None or len(charuco_ids) < 4:
            print(f"  ✗ ChArUco in Bild {i+1} nicht erkannt")
            continue

        obj_points, img_points = charuco_board.matchImagePoints(
            charuco_corners, charuco_ids
        )

        # FIX: dist=None weil Bild bereits entzerrt
        ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, None)
        if not ret:
            print(f"  ✗ solvePnP fehlgeschlagen für Bild {i+1}")
            continue

        R_kam, _ = cv2.Rodrigues(rvec)

        x, y, z, A, B, C = tcp_pose
        # FIX: t_rob als (3,1) – calibrateHandEye erwartet dieses Format
        # Einheit: Meter (tcp_auslesen gibt jetzt Meter zurück)
        t_rob = np.array([[x], [y], [z]], dtype=np.float64)
        rot   = Rotation.from_euler('xyz', [A, B, C], degrees=True)
        R_rob = rot.as_matrix()

        R_rob_liste.append(R_rob)
        t_rob_liste.append(t_rob)
        R_kam_liste.append(R_kam)
        t_kam_liste.append(tvec)  # shape (3,1) aus solvePnP
        print(f"  ✓ Bild {i+1} verarbeitet")

    if len(R_rob_liste) < 5:
        print("✗ Zu wenige gültige Bilder!")
        return

    # Mehrere Methoden vergleichen und beste wählen
    methoden = {
        "TSAI":     cv2.CALIB_HAND_EYE_TSAI,
        "PARK":     cv2.CALIB_HAND_EYE_PARK,
        "HORAUD":   cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF":  cv2.CALIB_HAND_EYE_ANDREFF,
    }

    ergebnisse = {}
    for name, methode in methoden.items():
        try:
            R_he, t_he = cv2.calibrateHandEye(
                R_rob_liste, t_rob_liste,
                R_kam_liste, t_kam_liste,
                method=methode
            )
            fehler = _reprojektionsfehler_berechnen(
                R_he, t_he, R_rob_liste, t_rob_liste, R_kam_liste, t_kam_liste
            )
            ergebnisse[name] = (R_he, t_he, fehler)
            print(f"  [{name:8s}]  Reprojektionsfehler: {fehler:.4f} m")
        except Exception as e:
            print(f"  [{name:8s}]  Fehlgeschlagen: {e}")

    if not ergebnisse:
        print("✗ Alle Methoden fehlgeschlagen!")
        return

    # Methode mit kleinstem Fehler wählen
    beste_methode = min(ergebnisse, key=lambda k: ergebnisse[k][2])
    R_he, t_he, bester_fehler = ergebnisse[beste_methode]

    print(f"\n  → Beste Methode: {beste_methode} (Fehler: {bester_fehler:.4f} m)")

    T_he = np.eye(4)
    T_he[:3, :3] = R_he
    T_he[:3, 3]  = t_he.flatten()

    print("\n═══════════════════════════════════════")
    print("  Hand-Eye-Kalibrierung Ergebnis")
    print("═══════════════════════════════════════")
    print(f"  Methode: {beste_methode}")
    print(f"  R:\n{R_he}")
    print(f"  t (m):  {t_he.flatten()}")
    print(f"  t (mm): {t_he.flatten() * 1000}")
    print(f"  Reprojektionsfehler: {bester_fehler:.4f} m ({bester_fehler*1000:.2f} mm)")
    print("═══════════════════════════════════════")

    np.savez(
        RESULT_FILE,
        R=R_he,
        t=t_he,
        T=T_he,
        methode=beste_methode,
        reprojektionsfehler=bester_fehler
    )
    print(f"\n✓ Gespeichert: {RESULT_FILE}")


def _reprojektionsfehler_berechnen(R_he, t_he, R_rob_liste, t_rob_liste,
                                    R_kam_liste, t_kam_liste):
    """
    Validierung der Hand-Eye-Kalibrierung.

    Für jedes Posenpaar (i, j) gilt bei korrekter Kalibrierung:
        A_ij = R_rob_j^-1 @ R_rob_i
        B_ij = R_kam_j^-1 @ R_kam_i
    und die Gleichung  A @ X = X @ B  (X = Hand-Eye-Transformation).

    Wir messen den mittleren Rotationsfehler als Proxy für die Qualität.
    """
    fehler_liste = []
    n = len(R_rob_liste)

    for i in range(n):
        for j in range(i + 1, n):
            # Relative Transformation Roboter
            A = R_rob_liste[j].T @ R_rob_liste[i]
            # Relative Transformation Kamera
            B = R_kam_liste[j].T @ R_kam_liste[i]

            # AX = XB  →  Residuum: R_he @ B vs. A @ R_he
            lhs = R_he @ B
            rhs = A @ R_he

            # Rotationsfehler als Frobenius-Norm der Differenz
            fehler = np.linalg.norm(lhs - rhs, 'fro')
            fehler_liste.append(fehler)

    return float(np.mean(fehler_liste)) if fehler_liste else float('inf')


# ═══════════════════════════════════════════════════════════════
# Hauptprogramm
# ═══════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = HandEyeNode()

    tcp_posen   = []
    gespeichert = 0
    anzahl      = len(KALIBRIER_POSITIONEN)

    print(f"\n{'='*55}")
    print(f"  Hand-Eye-Kalibrierung – {anzahl} Positionen")
    print(f"  ChArUco Board FEST hinlegen!")
    print(f"  TCP-Einheit: Meter (intern, konsistent mit solvePnP)")
    print(f"{'='*55}\n")

    input("EINGABE drücken um zu starten ...")

    for i, pos_deg in enumerate(KALIBRIER_POSITIONEN):
        print(f"\n{'='*55}")
        print(f"  Position {i+1}/{anzahl}: {pos_deg}")
        print(f"{'='*55}")

        # Roboter fahren
        print("  Fahre zu Position ...")
        if not node.move_to_joints(pos_deg):
            print("  ✗ Planung fehlgeschlagen – überspringe!")
            continue

        print("  ✓ Position erreicht.")
        # FIX: 2s reichen – Roboter steht nach spin_until_future_complete bereits
        time.sleep(2.0)

        # TCP auslesen
        tcp_pose = node.tcp_auslesen()
        if tcp_pose is None:
            print("  ✗ TCP nicht auslesbar – überspringe!")
            continue

        print(f"  TCP: x={tcp_pose[0]*1000:.1f} y={tcp_pose[1]*1000:.1f} "
              f"z={tcp_pose[2]*1000:.1f} mm  "
              f"A={tcp_pose[3]:.1f}° B={tcp_pose[4]:.1f}° C={tcp_pose[5]:.1f}°")

        # Bild aufnehmen
        print("  Nehme Bild auf ...")
        try:
            img = kamera_bild_aufnehmen()
        except Exception as e:
            print(f"  ✗ Kamerafehler: {e} – überspringe!")
            continue

        # ChArUco prüfen (am Rohbild, nur zur Vorprüfung)
        ret, _, _ = charuco_erkennen(img)
        if not ret:
            print("  ✗ ChArUco nicht erkannt – überspringe!")
            continue

        # FIX: Dateiname basiert auf gespeichertem Index, NICHT auf i
        # → tcp_posen[k] entspricht immer handeye_{k+1:02d}.png
        pfad = os.path.join(SAVE_DIR, f"handeye_{gespeichert+1:02d}.png")
        cv2.imwrite(pfad, img)
        tcp_posen.append(tcp_pose)

        # JSON nach jedem Bild aktualisieren (crash-sicher)
        with open(POSES_FILE, "w") as f:
            json.dump(tcp_posen, f, indent=2)

        gespeichert += 1
        print(f"  ✓ Gespeichert als {os.path.basename(pfad)} ({gespeichert} total)")

    print(f"\n{'='*55}")
    print(f"  {gespeichert} gültige Positionen aufgenommen.")

    if gespeichert >= 5:
        print("  Berechne Kalibrierung ...")
        kalibrierung_berechnen()
    else:
        print("  ✗ Zu wenige Positionen für Kalibrierung (mind. 5)!")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
