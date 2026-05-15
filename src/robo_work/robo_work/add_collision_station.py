#!/usr/bin/env python3
"""
Lädt die Prozessstation als Kollisionsobjekt in die MoveIt2 Planning Scene.
Ausführen NACHDEM das Launch File gestartet ist.

Starten:
    python3 add_collision_station.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
import pyassimp
import time


# ── Einstellungen ─────────────────────────────────────────────
STL_PATH = "/home/lukas/GreifpunktStudien/src/robo_work/meshes/Tisch.stl"

# Position der Station im World-Frame (in Metern)
STATION_POSITION = [10.0, 10.0, 10.0]   # ← x, y, z anpassen
STATION_ROTATION = [0.0, 0.0, 0.0, 1.0]  # ← Quaternion x, y, z, w


class AddCollisionStation(Node):
    def __init__(self):
        super().__init__('add_collision_station')
        self.pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

    def load_stl(self, stl_path):
        self.get_logger().info(f'Lade STL: {stl_path}')
        
        mesh = Mesh()
        
        with pyassimp.load(stl_path) as scene:
            for vertex in scene.meshes[0].vertices:
                point = Point()
                point.x = float(vertex[0])
                point.y = float(vertex[1])
                point.z = float(vertex[2])
                mesh.vertices.append(point)

            for face in scene.meshes[0].faces:
                triangle = MeshTriangle()
                triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                mesh.triangles.append(triangle)

        self.get_logger().info(f'STL geladen: {len(mesh.vertices)} Vertices')
        return mesh

    def add_station(self):
        """Station zur MoveIt2 Planning Scene hinzufügen."""
        mesh = self.load_stl(STL_PATH)

        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "prozessstation"

        # Pose (Position + Rotation)
        pose = Pose()
        pose.position.x = STATION_POSITION[0]
        pose.position.y = STATION_POSITION[1]
        pose.position.z = STATION_POSITION[2]
        pose.orientation.x = STATION_ROTATION[0]
        pose.orientation.y = STATION_ROTATION[1]
        pose.orientation.z = STATION_ROTATION[2]
        pose.orientation.w = STATION_ROTATION[3]

        obj.meshes.append(mesh)
        obj.mesh_poses.append(pose)
        obj.operation = CollisionObject.ADD

        self.pub.publish(obj)
        self.get_logger().info('✔ Prozessstation zur Planning Scene hinzugefügt!')

    def remove_station(self):
        """Station aus der Planning Scene entfernen."""
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "prozessstation"
        obj.operation = CollisionObject.REMOVE
        self.pub.publish(obj)
        self.get_logger().info('✔ Prozessstation entfernt.')


def main():
    rclpy.init()
    node = AddCollisionStation()

    # Kurz warten bis MoveIt2 bereit ist
    time.sleep(1.0)

    node.add_station()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
