# test_rebel_mover.py
import math
import rclpy
from robo_work.rebel_mover import RebelMover


def main():
    rclpy.init()
    mover = RebelMover()

    # ============================================================
    # TEST 1: Home-Position anfahren
    # ============================================================
    print("\n[TEST 1] Fahre auf Home-Position...")
    success = mover.move_to_home()
    print(f"  → {'OK' if success else 'FEHLGESCHLAGEN'}")
    input("  Enter drücken für nächsten Test...")

    # ============================================================
    # TEST 2: Gelenkwinkel-Test (sicher, weil bekannt)
    # ============================================================
    print("\n[TEST 2] Fahre Dummy-Gelenkwinkel an...")
    # leichte Beugung – nichts wildes
    success = mover.move_to_dummy_position([0, -30, 45, 0, 30, 0])
    print(f"  → {'OK' if success else 'FEHLGESCHLAGEN'}")
    input("  Enter drücken für nächsten Test...")

    # ============================================================
    # TEST 3: Kartesische Pose – Dummy-Greifpunkt
    # ============================================================
    print("\n[TEST 3] Fahre Dummy-Greifpunkt an (kartesisch)...")

    # Dummy-Werte – ANPASSEN je nach Arbeitsraum deines Roboters!
    # Diese Position sollte erreichbar sein und nichts treffen.
    x = 0.30   # 30 cm vor dem Roboter
    y = 0.00   # mittig
    z = 0.25   # 25 cm über Basis

    # Greifer zeigt nach unten (typische Greif-Orientierung)
    # Identitäts-Rotation = Greifer-Z zeigt wie base_link-Z (nach oben)
    # Für "nach unten greifen" drehen wir 180° um X-Achse:
    rotation_matrix = [
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1],
    ]

    success = mover.move_to_pose(x, y, z, rotation_matrix)
    print(f"  → {'OK' if success else 'FEHLGESCHLAGEN'}")
    input("  Enter drücken für nächsten Test...")

    # ============================================================
    # TEST 4: Pre-Grasp + Grasp (Anflug 10cm drüber, dann runter)
    # ============================================================
    print("\n[TEST 4] Pre-Grasp → Grasp-Sequenz...")

    print("  → Pre-Grasp (10 cm drüber)...")
    mover.move_to_pregrasp(x, y, z, rotation_matrix, approach_offset=0.10)
    input("  Enter drücken für Anfahrt...")

    print("  → Grasp-Pose...")
    mover.move_to_pose(x, y, z, rotation_matrix)
    input("  Enter drücken für Rückzug...")

    print("  → Wieder hoch...")
    mover.move_to_pregrasp(x, y, z, rotation_matrix, approach_offset=0.10)

    # ============================================================
    # Zurück auf Home
    # ============================================================
    print("\n[ENDE] Fahre zurück auf Home...")
    mover.move_to_home()

    mover.destroy_node()
    rclpy.shutdown()
    print("Fertig.")


if __name__ == "__main__":
    main()