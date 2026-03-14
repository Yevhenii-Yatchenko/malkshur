from pymavlink import mavutil
import time

# Підключаємося до Pixhawk через MAVLink
master = mavutil.mavlink_connection('/dev/ttyUSB0')

# Очікуємо, поки з'єднаємося
master.wait_heartbeat()
print("Connected to Pixhawk")

def send_position(x, y, z, roll=0, pitch=0, yaw=0):
    """
    Передає координати дрона у систему EKF через MAVLink.
    """
    master.mav.vision_position_estimate_send(
        int(time.time() * 1e6),  # Час у мікросекундах
        x, y, z,                 # Координати дрона
        roll, pitch, yaw          # Орієнтація (опціонально)
    )

def check_vision_data():
    """Отримує повідомлення vision_position_estimate, яке приймає ArduPilot"""
    msg = master.recv_match(type='VISION_POSITION_ESTIMATE', blocking=True, timeout=2)
    if msg:
        print(f"Received Vision Position: X={msg.x}, Y={msg.y}, Z={msg.z}")
    else:
        print("Pixhawk не отримує Vision Position Estimate!")

# Симуляція руху дрона
master.set_mode("GUIDED")

while True:
    x, y, z = 1.2, -0.5, -2.0  # Координати дрона, отримані з камери
    send_position(x, y, z)
    time.sleep(0.1)
    check_vision_data()