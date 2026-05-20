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
    

    # ============================================================
    # TEST 2: Gelenkwinkel-Test (sicher, weil bekannt)
    # ============================================================
    print("\n[TEST 2] Fahre Dummy-Gelenkwinkel an...")
    # leichte Beugung – nichts wildes
    success = mover.move_to_dummy_position([0, -79, 139, 0, 82, 0])
    print(f"  → {'OK' if success else 'FEHLGESCHLAGEN'}")
   

    # ============================================================
    # TEST 3: Kartesische Pose – Dummy-Greifpunkt
    # ============================================================
  
    # Greifer zeigt nach unten (typische Greif-Orientierung)
    

    # ============================================================
    # TEST 4: Pre-Grasp + Grasp (Anflug 10cm drüber, dann runter)
    # ============================================================
    

    # ============================================================
    # Zurück auf Home
    # ============================================================
   
    

    mover.destroy_node()
    rclpy.shutdown()
    print("Fertig.")


if __name__ == "__main__":
    main()