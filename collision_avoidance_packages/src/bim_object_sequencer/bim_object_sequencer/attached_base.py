#!/usr/bin/env python3
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    QoSHistoryPolicy,
)

import ifcopenshell
import ifcopenshell.geom
import numpy as np
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import Mesh as MeshMsg, MeshTriangle
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, String


class IfcWallStandardCaseParser(Node):
    """
    Loads IfcWallStandardCase meshes from the IFC and publishes them as
    ADDed CollisionObjects on /collision_object, in the 'world' frame.

    Notes:
    - No OBJ saving.
    - IFC file is resolved from the bim_object_sequencer package share:
      resource/file.ifc
    """

    def __init__(self):
        super().__init__('ifc_wall_standard_case_parser')

        # QoS for collision objects (latched + reliable)
        qos_attached = QoSProfile(depth=1)
        qos_attached.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_attached.reliability = ReliabilityPolicy.RELIABLE
        qos_attached.history = QoSHistoryPolicy.KEEP_LAST

        # Publish CollisionObject ADD to world instead of AttachedCollisionObject
        self._pub_co = self.create_publisher(
            CollisionObject, 'collision_object', qos_attached
        )

        # QoS for announcing pre-attached IDs: latched + reliable, keep a decent history
        qos_ids = QoSProfile(depth=200)
        qos_ids.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_ids.reliability = ReliabilityPolicy.RELIABLE
        qos_ids.history = QoSHistoryPolicy.KEEP_LAST
        self._pub_pre_attached = self.create_publisher(
            String, '/bim/pre_attached_id', qos_ids
        )

        # Resolve IFC from bim_object_sequencer resources
        pkg_share = get_package_share_directory('bim_object_sequencer')
        self.ifc_file = os.path.join(pkg_share, 'resource', 'file.ifc')
        self.get_logger().info(f'Using IFC file: {self.ifc_file}')

    def _wait_for_pre_attached_subscriber(self, timeout_sec: float = 3.0):
        """Wait briefly for at least one subscriber on /bim/pre_attached_id to match."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._pub_pre_attached.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f"/bim/pre_attached_id subs: {self._pub_pre_attached.get_subscription_count()}"
        )

    def parse_and_attach(self):
        # Open IFC
        ifc = ifcopenshell.open(self.ifc_file)

        # Configure geometry settings
        settings = ifcopenshell.geom.settings()
        try:
            settings.set(settings.USE_PYTHON_OPENCASCADE, True)
        except AttributeError:
            self.get_logger().warn(
                "Python-OpenCASCADE bindings not found—using default geometry engine."
            )

        # Export in world coordinates so vertices are already placed
        try:
            settings.set(settings.USE_WORLD_COORDS, True)
        except AttributeError:
            self.get_logger().info(
                "World-coordinate export not supported; verts may need manual transform."
            )

        # Optional tessellation controls (best-effort)
        try:
            settings.set(settings.LINEAR_DEFLECTION, 0.001)
            settings.set(settings.ANGULAR_DEFLECTION, 0.1)
        except Exception:
            pass

        # ---- Ensure a subscriber is matched before we announce IDs ----
        self._wait_for_pre_attached_subscriber(timeout_sec=3.0)

        walls = list(ifc.by_type('IfcWallStandardCase'))
        if not walls:
            self.get_logger().warn("No IfcWallStandardCase entities found in the IFC.")
            # Still return 0 so main() exits with 1 (as before)
            return 0

        # Build & publish a CollisionObject per wall
        for wall in walls:
            shape = ifcopenshell.geom.create_shape(settings, wall)
            geom = shape.geometry

            verts = np.array(geom.verts).reshape(-1, 3)
            faces = np.array(geom.faces).reshape(-1, 3)

            mesh_msg = MeshMsg()
            mesh_msg.vertices.extend(
                Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in verts
            )
            mesh_msg.triangles.extend(
                MeshTriangle(vertex_indices=[int(tri[0]), int(tri[1]), int(tri[2])])
                for tri in faces
            )

            # Create the CollisionObject (ADD to world)
            co = CollisionObject()
            co.id = wall.GlobalId
            co.header = Header(frame_id='world')  # keep world frame
            co.meshes = [mesh_msg]
            co.mesh_poses = [Pose()]  # identity pose in 'world'
            co.operation = CollisionObject.ADD  # add to planning scene

            self._pub_co.publish(co)
            # Announce this ID as pre-attached (so Sequencer can skip it)
            self._pub_pre_attached.publish(String(data=wall.GlobalId))
            self.get_logger().info(f'Added wall {wall.GlobalId} as a world collision object')

        # ---- ADDITION: also add these explicit IFC elements by GlobalId ----
        target_guids = [
            # "0l_dkEysv1awJx6pGbP4u1",
            "0l_dkEysv1awJx6pGbP4u7",
            "1PtCepLo5EbPKJfissQWLf",
            "1PtCepLo5EbPKJfissQWGQ",
            "1PtCepLo5EbPKJfissQWU1",
            "1PtCepLo5EbPKJfissQWTe",    # 5
            # "0l_dkEysv1awJx6pGbP4uo",
            "3zvUbI0Ef668TWfEjV97LU",
            "3zvUbI0Ef668TWfEjV97LN",
            "3zvUbI0Ef668TWfEjV97LJ",
            "3zvUbI0Ef668TWfEjV97Lb",    # 10
            "3zvUbI0Ef668TWfEjV97LQ",
            "3zvUbI0Ef668TWfEjV97LT",
            "3zvUbI0Ef668TWfEjV97LH",
            "0l_dkEysv1awJx6pGbP4zd",
            "0l_dkEysv1awJx6pGbP4$k",    # 15
            "3zvUbI0Ef668TWfEjV97Ld",
            "0l_dkEysv1awJx6pGbP4wD",
            "0l_dkEysv1awJx6pGbP4zb",
        ]

        for guid in target_guids:
            ent = ifc.by_guid(guid)
            if ent is None:
                self.get_logger().warn(f"GUID not found in IFC: {guid}")
                continue

            try:
                shape = ifcopenshell.geom.create_shape(settings, ent)
            except Exception as e:
                self.get_logger().warn(f"Geometry creation failed for {guid}: {e}")
                continue

            geom = shape.geometry
            verts = np.array(geom.verts).reshape(-1, 3)
            faces = np.array(geom.faces).reshape(-1, 3)

            if verts.size == 0 or faces.size == 0:
                self.get_logger().warn(f"No triangulated mesh for {guid}; skipping.")
                continue

            mesh_msg = MeshMsg()
            mesh_msg.vertices.extend(
                Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in verts
            )
            mesh_msg.triangles.extend(
                MeshTriangle(vertex_indices=[int(tri[0]), int(tri[1]), int(tri[2])])
                for tri in faces
            )

            co = CollisionObject()
            co.id = guid
            co.header = Header(frame_id='world')
            co.meshes = [mesh_msg]
            co.mesh_poses = [Pose()]
            co.operation = CollisionObject.ADD

            self._pub_co.publish(co)
            # Announce this ID as pre-attached
            self._pub_pre_attached.publish(String(data=guid))
            self.get_logger().info(f'Added element {guid} as a world collision object')
        # ---- END ADDITION ----

        return len(walls)


def main(args=None):
    rclpy.init(args=args)
    node = IfcWallStandardCaseParser()
    count = node.parse_and_attach()

    # Keep the node alive briefly so TRANSIENT_LOCAL data can be delivered
    t_end = time.time() + 5.0  # ~5s grace period
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    # exit code 0 if we attached at least one wall, else 1
    sys.exit(0 if count else 1)


if __name__ == '__main__':
    main()

