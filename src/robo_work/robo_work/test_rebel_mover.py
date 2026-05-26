# test_rebel_mover.py
import math
import time
import rclpy
from robo_work.rebel_mover import RebelMover
from scipy.spatial.transform import Rotation as R

def main():
    rclpy.init()
    mover = RebelMover()
    gripPointPos = [0.0, -77, 138.0, 0.0, 81.0, 0.0]
    PreGraspPos = [0.0, 25.5, 69.9, 0.0, 81.0, 0.0]
    GraspPos = [0.0, 29.5, 67.5, 0.0, 81.0, 0.0]
    checkLabelPos = [66, 23.6, 74.5, 0.0, 79.9, 66]
    traySortPause1 = [66, 23.6, 74.5, 0.0, 79.9, 66]
    traySortPause2 = [-50, -25.8, 74.5, 0.0, 80.0,-23.0]
    trayDummy1 = [-83.5,28.0,62.6,0.0,80.0,0.0]

    mover.move_to_dummy_position(gripPointPos)
    time.sleep(0.5)
    mover.move_to_dummy_position(PreGraspPos)
    time.sleep(0.5)
    mover.move_to_dummy_position(GraspPos)
    time.sleep(0.5)
    mover.move_to_dummy_position(checkLabelPos)
    time.sleep(0.5)
    mover.move_to_dummy_position(traySortPause1)
    time.sleep(0.5)
    mover.move_to_dummy_position(traySortPause2)
    time.sleep(0.5)
    mover.move_to_dummy_position(trayDummy1)
    time.sleep(0.5)
    mover.move_to_dummy_position(gripPointPos)
    

    mover.destroy_node()
    rclpy.shutdown()
    print("Fertig.")


if __name__ == "__main__":
    main()