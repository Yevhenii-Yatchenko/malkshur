#!/usr/bin/env python3
"""Check what altitude messages Gazebo is sending"""

import time
import os
from pymavlink import mavutil

# Connect to Gazebo
if os.environ.get('USE_GAZEBO'):
    host = os.environ.get('MAVLINK_HOST', 'host.docker.internal')
    port = int(os.environ.get('MAVLINK_PORT', '5763'))
    connection_string = f'tcp:{host}:{port}'
else:
    connection_string = '/dev/ttyUSB0'

print(f"Connecting to: {connection_string}")
master = mavutil.mavlink_connection(connection_string, baud=57600)
master.wait_heartbeat()
print("Connected!")

# Request altitude messages
message_ids = [
    mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
]

for msg_id in message_ids:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        100000,  # 10Hz
        0, 0, 0, 0, 0
    )

print("\nMonitoring altitude messages for 5 seconds...")
start_time = time.time()
message_counts = {}

while time.time() - start_time < 5:
    msg = master.recv_match(blocking=False)
    if msg:
        msg_type = msg.get_type()

        # Track message counts
        if msg_type not in message_counts:
            message_counts[msg_type] = 0
        message_counts[msg_type] += 1

        # Display altitude-related messages
        if msg_type == 'VFR_HUD':
            print(f"VFR_HUD: alt={msg.alt:.3f}m, climb={msg.climb:.3f}m/s")
        elif msg_type == 'GLOBAL_POSITION_INT':
            print(f"GLOBAL_POSITION_INT: alt={msg.alt/1000:.3f}m, relative_alt={msg.relative_alt/1000:.3f}m")
        elif msg_type == 'LOCAL_POSITION_NED':
            print(f"LOCAL_POSITION_NED: z={-msg.z:.3f}m (NED, so positive is up), vz={msg.vz:.3f}m/s")
        elif msg_type == 'ALTITUDE':
            print(f"ALTITUDE: altitude_local={msg.altitude_local:.3f}m, altitude_relative={msg.altitude_relative:.3f}m")

    time.sleep(0.01)

print("\n\nMessage counts:")
for msg_type, count in sorted(message_counts.items()):
    print(f"  {msg_type}: {count}")