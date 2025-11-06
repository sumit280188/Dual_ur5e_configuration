#!/usr/bin/env python3
import os
import json  # <-- added
import numpy as np
import rclpy
import ifcopenshell
import ifcopenshell.geom
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh as MeshMsg, MeshTriangle, SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject
from ament_index_python.packages import get_package_share_directory  # <-- added


class Spawner(Node):
    """
    Receives spawn requests (index:i), loads the i-th IFC product mesh,
    stages it, and publishes as a CollisionObject ADD.

    Publishes:
      - /spawned_collision_object_id : the GlobalId of the last spawned object (latched)
      - /bim/last_spawned_id         : legacy (non-latched)
    """

    def __init__(self):
        super().__init__('bim_object_spawner')

        self.ifc_path = self.declare_parameter(
            'ifc_path',
            '/home/robotik/workspace/src/collision_avoidance_packages/src/bim_object_sequencer/resource/file.ifc'
        ).get_parameter_value().string_value
        self.world_frame = self.declare_parameter(
            'world_frame', 'world'
        ).get_parameter_value().string_value
        self.max_objects = self.declare_parameter(
            'max_objects', 0
        ).get_parameter_value().integer_value
        self.unit_scale = float(
            self.declare_parameter('unit_scale', 1.0).get_parameter_value().double_value
        )
        # Let the JSON filename be configurable (default same as sequencer)
        self.json_filename = self.declare_parameter(
            'json_filename', 'MontageSequenz_Gesamt.json'
        ).get_parameter_value().string_value

        self.stage_xyz = np.array([-1.0, 0.0, 0.0], dtype=float)
        self.extra_yaw_deg = 90.0
        # Ensure lowest point is above z=0 by this clearance (meters)
        self.floor_clearance = float(
            self.declare_parameter('floor_clearance', 0.001).get_parameter_value().double_value
        )

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.co_pub = self.create_publisher(CollisionObject, 'collision_object', qos)
        self.spawn_sub = self.create_subscription(
            String, '/bim/spawn_request', self._on_spawn_request, 10
        )

        # Legacy publisher
        self.last_spawned_pub = self.create_publisher(String, '/bim/last_spawned_id', 10)

        # Latched publisher for attach_detach node
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self.spawned_id_pub = self.create_publisher(String, '/spawned_collision_object_id', latched_qos)

        self.attach_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.attach_sub = self.create_subscription(String, '/bim/attach_request', self._on_attach_request, 10)
        self.detach_sub = self.create_subscription(String, '/bim/detach_request', self._on_detach_request, 10)

        # --- Floor: publish now and once more shortly after startup ---
        self._publish_floor()
        self._floor_timer = self.create_timer(0.5, self._republish_floor_once)

        self._load_ifc_meshes()             # builds self.ids / self.meshes from ALL products
        self._maybe_apply_json_order()      # NEW: reorder to match JSON MontageIndex → IfcGUID

        self.get_logger().info(
            f"Spawner ready. Loaded {len(self.ids)} IFC products from: {self.ifc_path}"
        )

    def _publish_floor(self):
        floor = CollisionObject()
        floor.header = Header(frame_id=self.world_frame)
        floor.id = "floor"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [4.0, 4.0, 0.01]
        floor_pose = Pose()
        floor_pose.position.z = -0.02
        floor_pose.orientation.w = 1.0
        floor.primitives = [box]
        floor.primitive_poses = [floor_pose]
        floor.operation = CollisionObject.ADD
        self.co_pub.publish(floor)
        self.get_logger().info("Published floor collision object.")

    def _republish_floor_once(self):
        self._publish_floor()
        # cancel this one-shot timer
        self._floor_timer.cancel()

    def _load_ifc_meshes(self):
        if not os.path.exists(self.ifc_path):
            raise FileNotFoundError(f"IFC file not found: {self.ifc_path}")

        model = ifcopenshell.open(self.ifc_path)

        settings = ifcopenshell.geom.settings()
        settings.set(settings.USE_WORLD_COORDS, True)
        try:
            settings.set(settings.LINEAR_DEFLECTION, 0.001)
            settings.set(settings.ANGULAR_DEFLECTION, 0.1)
        except Exception:
            pass

        # Load ALL IFC products, not only IfcFlowSegment
        ents = list(model.by_type('IfcProduct'))
        if not ents:
            raise RuntimeError("No IfcProduct found in IFC.")

        if self.max_objects > 0:
            ents = ents[: int(self.max_objects)]

        self.meshes = []
        self.ids = []

        for ent in ents:
            try:
                shape = ifcopenshell.geom.create_shape(settings, ent)
                geom = shape.geometry
                if not geom.verts or not geom.faces:
                    self.get_logger().warn(
                        f"IFC entity {ent.GlobalId} has no valid geometry; skipping."
                    )
                    continue
                verts = np.array(geom.verts, dtype=float).reshape(-1, 3)
                faces = np.array(geom.faces, dtype=int).reshape(-1, 3)
                if self.unit_scale != 1.0:
                    verts *= self.unit_scale
                self.meshes.append((verts, faces))
                self.ids.append(ent.GlobalId)
            except Exception as e:
                self.get_logger().warn(
                    f"Unsupported or invalid IFC entity {getattr(ent, 'GlobalId', 'UNKNOWN')} "
                    f"({ent.is_a()}), skipping. Error: {e}"
                )
                continue

    def _maybe_apply_json_order(self):
        """
        If resource/MontageSequenz_Gesamt.json exists (installed share path),
        reorder self.ids/self.meshes to match its MontageIndex → IfcGUID order.
        """
        try:
            pkg_share = get_package_share_directory('bim_object_sequencer')
            json_path = os.path.join(pkg_share, 'resource', self.json_filename)
        except Exception:
            # If the package isn't installed (dev run), try source path as a fallback
            json_path = os.path.join(
                os.path.dirname(os.path.dirname(self.ifc_path)),  # .../bim_object_sequencer
                'resource',
                self.json_filename
            )

        if not os.path.exists(json_path):
            self.get_logger().info(f"No JSON order found at {json_path}; keeping IFC enumeration order.")
            return

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse JSON order: {e}; keeping IFC enumeration order.")
            return

        # Build GUID -> (verts, faces) lookup from the already prepared meshes
        gid_to_mesh = {gid: mf for gid, mf in zip(self.ids, self.meshes)}

        ordered_pairs = []
        for row in data:
            guid = row.get('IfcGUID') or row.get('ifcGuid')
            if guid is None:
                continue
            try:
                idx = int(row.get('MontageIndex'))
            except Exception:
                continue
            ordered_pairs.append((idx, guid))

        if not ordered_pairs:
            self.get_logger().warn("JSON had no valid IfcGUID + MontageIndex entries; keeping IFC enumeration order.")
            return

        ordered_pairs.sort(key=lambda x: (x[0], x[1]))
        ordered_ids = []
        ordered_meshes = []

        missing = 0
        for _, gid in ordered_pairs:
            mf = gid_to_mesh.get(gid)
            if mf is None:
                missing += 1
                continue
            ordered_ids.append(gid)
            ordered_meshes.append(mf)

        if not ordered_ids:
            self.get_logger().warn("None of the JSON GUIDs had geometry in the IFC; keeping IFC enumeration order.")
            return

        self.ids = ordered_ids
        self.meshes = ordered_meshes
        if missing > 0:
            self.get_logger().warn(f"{missing} GUIDs from JSON not found with geometry; they were skipped.")
        self.get_logger().info(f"Applied JSON order. {len(self.ids)} spawnable GUIDs will match the sequencer.")

    def _align_long_axis_and_stage(self, verts: np.ndarray) -> np.ndarray:
        c = verts.mean(axis=0)
        V0 = verts - c
        H = (V0.T @ V0) / max(len(V0), 1)
        w, E = np.linalg.eigh(H)
        E = E[:, np.argsort(w)[::-1]]
        if np.linalg.det(E) < 0:
            E[:, -1] *= -1.0
        V_rot = V0 @ E
        yaw = np.deg2rad(self.extra_yaw_deg)
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cos_y, -sin_y, 0.0],
                       [sin_y,  cos_y, 0.0],
                       [0.0,    0.0,   1.0]], dtype=float)
        V_yaw = V_rot @ Rz
        V_new = V_yaw + self.stage_xyz

        # --- NEW: lift so the lowest vertex clears the floor by floor_clearance ---
        min_z = float(V_new[:, 2].min())
        dz = self.floor_clearance - min_z
        if dz > 0.0:
            V_new[:, 2] += dz

        return V_new

    def _on_spawn_request(self, msg: String):
        if not msg.data.startswith('index:'):
            return
        try:
            idx = int(msg.data.split(':')[-1])
        except Exception:
            self.get_logger().warn(f"Bad spawn request payload: '{msg.data}'")
            return

        if idx < 0 or idx >= len(self.ids):
            self.get_logger().warn(
                f"Requested index {idx} out of range (0..{len(self.ids)-1})."
            )
            return

        gid = self.ids[idx]
        verts, faces = self.meshes[idx]

        verts_stage = self._align_long_axis_and_stage(verts)

        mesh_msg = MeshMsg()
        for v in verts_stage:
            mesh_msg.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for tri in faces:
            mesh_msg.triangles.append(
                MeshTriangle(vertex_indices=[int(tri[0]), int(tri[1]), int(tri[2])])
            )

        co = CollisionObject()
        co.id = gid
        co.header = Header(frame_id=self.world_frame)
        co.meshes = [mesh_msg]
        co.mesh_poses = [Pose()]
        co.operation = CollisionObject.ADD

        self.co_pub.publish(co)
        self.get_logger().info(
            f"Published IFC product index {idx} ({gid}) staged at {tuple(self.stage_xyz)}."
        )

        # Publish the GlobalId on both topics
        last_msg = String()
        last_msg.data = gid
        self.last_spawned_pub.publish(last_msg)
        self.spawned_id_pub.publish(last_msg)  # <-- for attach_detach

    def _on_attach_request(self, msg: String):
        try:
            parts = dict(p.split(':', 1) for p in msg.data.split(','))
            gid = parts['id'].strip()
            link = parts.get('link', 'tool0').strip()
        except Exception:
            self.get_logger().warn(f"Bad attach payload: '{msg.data}'")
            return

        if gid not in self.ids:
            self.get_logger().warn(f"Attach requested for unknown id: {gid}")
            return

        aco = AttachedCollisionObject()
        aco.object.id = gid
        aco.link_name = link
        aco.object.operation = CollisionObject.ADD
        self.attach_pub.publish(aco)
        self.get_logger().info(f"Attached {gid} to link '{link}'.")

    def _on_detach_request(self, msg: String):
        try:
            parts = dict(p.split(':', 1) for p in msg.data.split(','))
            gid = parts['id'].strip()
            place = parts.get('place', None)
        except Exception:
            self.get_logger().warn(f"Bad detach payload: '{msg.data}'")
            return

        if gid not in self.ids:
            self.get_logger().warn(f"Detach requested for unknown id: {gid}")
            return

        aco = AttachedCollisionObject()
        aco.object.id = gid
        aco.object.operation = CollisionObject.REMOVE
        self.attach_pub.publish(aco)
        self.get_logger().info(f"Detached {gid} from robot.")

        if place is not None:
            try:
                vals = [float(v) for v in place.strip().split()]
                x, y, z, qx, qy, qz, qw = vals
            except Exception:
                self.get_logger().warn(f"Bad place pose: '{place}' (expected 7 floats)")
                return

            idx = self.ids.index(gid)
            verts, faces = self.meshes[idx]

            mesh_msg = MeshMsg()
            for v in verts:
                mesh_msg.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
            for tri in faces:
                mesh_msg.triangles.append(MeshTriangle(vertex_indices=[int(tri[0]), int(tri[1]), int(tri[2])]))

            co = CollisionObject()
            co.id = gid
            co.header = Header(frame_id=self.world_frame)
            co.meshes = [mesh_msg]

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            co.mesh_poses = [pose]
            co.operation = CollisionObject.ADD

            self.co_pub.publish(co)
            self.get_logger().info(f"Re-added {gid} to world at placement pose ({x:.3f},{y:.3f},{z:.3f}).")


def main():
    rclpy.init()
    node = Spawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

