## Launches Ur5e + Hand-e gripper

No need to create socat connection.

ros2 launch real_ur5e_hande_description ur5e_hande_control.launch.py   ur_type:=ur5e   robot_ip:=192.168.1.102   use_tool_communication:=false   tool_voltage:=24   tool_baud_rate:=115200   create_socat_tty:=true   use_fake_hardware:=false   launch_rviz:=true
