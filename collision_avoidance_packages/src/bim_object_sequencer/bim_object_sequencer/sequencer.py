#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from moveit_msgs.msg import AttachedCollisionObject
from ament_index_python.packages import get_package_share_directory

class Sequencer(Node):
    """
    Controls WHEN to place the next object.
    - Loads order strictly from resource/MontageSequenz_Gesamt.json (sorted by MontageIndex).
    - Sends spawn requests index:0,1,2...
    - Publishes the current target GlobalId on /bim/current_target_id for the picker.
    - Waits for /detached_collision_object (object released) to confirm, then advances.
    """

    def __init__(self):
        super().__init__('bim_object_sequencer')

        # Parameters (keep existing ones so launches don't break)
        self.ifc_path = self.declare_parameter(
            'ifc_path',
            '/home/robotik/workspace/src/collision_avoidance_packages/src/bim_object_sequencer/resource/file.ifc'
        ).get_parameter_value().string_value  # not used here, JSON is authoritative

        self.max_objects = self.declare_parameter('max_objects', 0).get_parameter_value().integer_value  # 0 = all
        self.json_filename = self.declare_parameter(
            'json_filename', 'MontageSequenz_Gesamt.json'
        ).get_parameter_value().string_value

        # Publishers / Subscribers
        self.spawn_pub = self.create_publisher(String, '/bim/spawn_request', 10)
        self.target_pub = self.create_publisher(String, '/bim/current_target_id', 10)

        self.detach_sub = self.create_subscription(
            AttachedCollisionObject, '/detached_collision_object', self._on_detach, 10
        )

        # NEW: subscribe to pre-attached IDs so we can skip them
        self.pre_attached_ids = set()
        qos_ids = QoSProfile(depth=200)
        qos_ids.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/bim/pre_attached_id', self._on_pre_attached_id, qos_ids
        )

        # Load ordered list of IDs strictly from JSON
        self._load_ids_from_json()
        self.index = 0
        self.waiting_gid = None

        # Kick off first object
        self._request_spawn(self.index)

    # ------------ JSON ordering ------------
    def _load_ids_from_json(self):
        # Resolve installed package share dir
        pkg_share = get_package_share_directory('bim_object_sequencer')
        json_path = os.path.join(pkg_share, 'resource', self.json_filename)

        if not os.path.exists(json_path):
            raise FileNotFoundError(f'JSON not found: {json_path}')

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            raise RuntimeError(f'Failed to parse JSON: {e}')

        cleaned = []
        for row in data:
            guid = row.get('IfcGUID') or row.get('ifcGuid')
            if not guid:
                continue
            try:
                idx = int(row.get('MontageIndex'))
            except Exception:
                # If MontageIndex is missing or malformed, treat as invalid entry
                continue
            cleaned.append((idx, guid))

        if not cleaned:
            raise RuntimeError('JSON contains no valid entries with IfcGUID + MontageIndex.')

        cleaned.sort(key=lambda x: (x[0], x[1]))
        guids = [g for _, g in cleaned]

        if self.max_objects and self.max_objects > 0:
            guids = guids[: int(self.max_objects)]

        if not guids:
            raise RuntimeError('After applying max_objects, no GUIDs remain to sequence.')

        self.ids = guids
        self.get_logger().info(f"Sequencer loaded {len(self.ids)} IDs from JSON order.")

    # ------------ pre-attached callback ------------
    def _on_pre_attached_id(self, msg: String):
        gid = (msg.data or '').strip()
        if gid:
            self.pre_attached_ids.add(gid)
            self.get_logger().debug(f"Marked pre-attached (skip): {gid}")

    # ------------ sequencing ------------
    def _request_spawn(self, i: int):
        # Skip over any IDs that were already attached by attached_base.py
        while i < len(self.ids) and self.ids[i] in self.pre_attached_ids:
            self.get_logger().info(f"Skipping pre-attached id: {self.ids[i]}")
            i += 1
        self.index = i

        if i >= len(self.ids):
            self.get_logger().info("Sequence complete. No more objects.")
            return

        self.waiting_gid = self.ids[i]

        # 1) Ask spawner to stage the i-th object
        msg = String()
        msg.data = f'index:{i}'
        self.spawn_pub.publish(msg)

        # 2) Publish the current target GlobalId for the picker to latch onto
        gid_msg = String()
        gid_msg.data = self.waiting_gid
        self.target_pub.publish(gid_msg)

        self.get_logger().info(f"Requested spawn for index {i} (id={self.waiting_gid}).")
        self.get_logger().info(f"Published current target id on /bim/current_target_id: {self.waiting_gid}")

    def _on_detach(self, msg: AttachedCollisionObject):
        obj_id = msg.object.id
        if self.waiting_gid is None or obj_id != self.waiting_gid:
            return

        self.get_logger().info(f"path planning {self.index} is complete")
        print(f"path planning {self.index} is complete")

        self.index += 1
        self.waiting_gid = None
        self._request_spawn(self.index)

def main():
    rclpy.init()
    node = Sequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

