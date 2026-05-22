#!/usr/bin/env python3
"""
Greifpunkt-Test  –  manuelle Koordinateneingabe
=================================================
Gibt den Greifpunkt von HappyPose ein, Roboter fährt:
  1. Sicherheitsposition (gleiche X/Y, Z + Sicherheitsabstand)
  2. Greifpunkt

Verwendung:
    ros2 run <package> greif_test
    oder:
    python3 greif_test.py

Eingabe in Metern (wie ROS2/MoveIt2 erwartet)!
HappyPose gibt mm aus → durch 1000 teilen.
"""

import rclpy
import numpy as np
from rebel_mover import RebelMover

# ── Konfiguration ──────────────────────────────────────────────
SICHERHEITS_OFFSET_M = 0.10   # 100mm über Greifpunkt anfahren
GREIFER_LAENGE_M     = 0.20   # 200mm Greiferlänge – Z-Offset


def main():
    rclpy.init()
    mover = RebelMover()

    print("\n" + "═"*55)
    print("  Greifpunkt-Test  –  manuelle Eingabe")
    print("═"*55)
    print("  HappyPose gibt mm aus → hier in Meter eingeben!")
    print("  Beispiel: x=290.1mm → 0.2901")
    print("═"*55 + "\n")

    while True:
        eingabe = input("  Greifpunkt  x y z [m]  (oder q): ").strip()

        if eingabe.lower() == "q":
            break

        parts = eingabe.replace(",", ".").split()
        if len(parts) != 3:
            print("  ⚠  Bitte genau 3 Werte eingeben: x y z\n")
            continue
        try:
            x, y, z = map(float, parts)
        except ValueError:
            print("  ⚠  Ungültige Zahlen.\n")
            continue

        # Greifer von oben → Rotationsmatrix "Greifer zeigt nach unten"
        # Z-Achse des Greifers zeigt nach unten (Welt-Z negativ)
        R_von_oben = np.array([
            [ 1,  0,  0],
            [ 0, -1,  0],
            [ 0,  0, -1]
        ], dtype=float)

        # Z-Offset für Greiferlänge
        z_greif = z - GREIFER_LAENGE_M

        print(f"\n  Greifpunkt:         x={x:.4f}  y={y:.4f}  z={z_greif:.4f} m")
        print(f"  Sicherheitspos.:    x={x:.4f}  y={y:.4f}  z={z_greif + SICHERHEITS_OFFSET_M:.4f} m")

        # ── Schritt 1: Sicherheitsposition ────────────────────
        print("\n  1️⃣   Fahre Sicherheitsposition an ...")
        ok = mover.move_to_pregrasp(x, y, z_greif, R_von_oben,
                                     approach_offset=SICHERHEITS_OFFSET_M)
        if not ok:
            print("  ✗  Planung fehlgeschlagen! Abbruch.\n")
            continue
        print("  ✓  Sicherheitsposition erreicht.")

        bestaetigung = input("\n  Sieht gut aus? Enter=Weiter  q=Abbruch: ").strip().lower()
        if bestaetigung == "q":
            print("  ↩  Abgebrochen.\n")
            continue

        # ── Schritt 2: Greifpunkt anfahren ────────────────────
        print("\n  2️⃣   Fahre Greifpunkt an ...")
        ok = mover.move_to_pose(x, y, z_greif, R_von_oben)
        if not ok:
            print("  ✗  Planung fehlgeschlagen! Abbruch.\n")
            continue
        print("  ✓  Greifpunkt erreicht.\n")

        weiter = input("  Nächsten Greifpunkt eingeben? Enter=Ja  q=Beenden: ").strip().lower()
        if weiter == "q":
            break

    print("\n  Home-Position anfahren ...")
    mover.move_to_home()
    print("  ✓  Fertig!\n")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
