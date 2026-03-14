# Jetson Nano UART Setup for TF-Luna LiDAR

## Finding UART Ports on Jetson Nano

### 1. List Available Serial Ports
```bash
# List all serial devices
ls -la /dev/tty*

# Common UART ports on Jetson Nano:
# /dev/ttyTHS1 - UART on J41 pins (hardware UART)
# /dev/ttyS0   - Debug console (usually occupied)
# /dev/ttyUSB* - USB-to-serial adapters
# /dev/ttyACM* - USB serial devices

# Check which serial ports exist
ls -la /dev/ttyTHS* /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

### 2. Hardware UART Pins on Jetson Nano (J41 Header)

The main UART available for use is on pins 8 and 10 of the J41 header:
- **Pin 8**: UART TX (Transmit)
- **Pin 10**: UART RX (Receive)
- **Pin 6**: GND (Ground)

This UART appears as `/dev/ttyTHS1` in the system.

### 3. Check UART Availability
```bash
# Check if UART is in use
sudo lsof /dev/ttyTHS1

# Check permissions
ls -l /dev/ttyTHS1

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Logout and login for changes to take effect
```

### 4. Test UART Port
```bash
# Install serial tools
sudo apt-get update
sudo apt-get install -y python3-serial minicom

# Test with Python
python3 -c "import serial; print(serial.Serial('/dev/ttyTHS1', 115200, timeout=1))"

# Or test with minicom
sudo minicom -D /dev/ttyTHS1 -b 115200
```

### 5. Disable Serial Console (if needed)

If `/dev/ttyTHS1` is being used by the system, disable it:

```bash
# Check if serial console is enabled
cat /proc/cmdline | grep ttyTHS1

# Disable serial console (if needed)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Or edit boot configuration
# sudo nano /boot/extlinux/extlinux.conf
# Remove "console=ttyTHS1,115200" from the kernel command line
```

### 6. Wire Connection for TF-Luna

**IMPORTANT**: TF-Luna operates at 3.3V logic levels, which matches Jetson Nano!

| TF-Luna Wire | Jetson J41 Pin | Description |
|--------------|----------------|-------------|
| Red (5V)     | Pin 2 or 4     | 5V Power    |
| Black (GND)  | Pin 6          | Ground      |
| White (RX)   | Pin 8          | UART TX     |
| Green (TX)   | Pin 10         | UART RX     |

**Note**: TX of TF-Luna connects to RX of Jetson, and vice versa!

### 7. Quick Test Script
```python
#!/usr/bin/env python3
import serial
import time

# Test if UART port is accessible
def test_uart_port(port='/dev/ttyTHS1'):
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"✓ Successfully opened {port}")
        
        # Check if data is being received
        print("Waiting for data (5 seconds)...")
        start_time = time.time()
        data_received = False
        
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"✓ Received {len(data)} bytes")
                data_received = True
                break
        
        if not data_received:
            print("✗ No data received - check connections")
        
        ser.close()
        return True
        
    except serial.SerialException as e:
        print(f"✗ Failed to open {port}: {e}")
        return False

if __name__ == "__main__":
    test_uart_port()
```

### 8. Alternative: USB-to-Serial Adapter

If the built-in UART is not available, use a USB-to-serial adapter:

```bash
# Connect USB-to-serial adapter
# It will appear as /dev/ttyUSB0 (or ttyUSB1, etc.)

# Find the adapter
dmesg | grep -i "usb.*serial"
ls -la /dev/ttyUSB*

# Update the lidar_sensor.py to use:
sensor = LidarSensor(port='/dev/ttyUSB0', baudrate=115200)
```

### 9. Troubleshooting

1. **Permission Denied**:
   ```bash
   # Add to dialout group
   sudo usermod -a -G dialout $USER
   # Or run with sudo (not recommended)
   ```

2. **Port Busy**:
   ```bash
   # Find what's using the port
   sudo lsof /dev/ttyTHS1
   # Kill the process if needed
   ```

3. **No Data Received**:
   - Check TX/RX are swapped correctly
   - Verify power connections (5V and GND)
   - Ensure TF-Luna is powered (red LED should be on)
   - Try different baud rate (default is 115200)

4. **Port Not Found**:
   ```bash
   # Check kernel messages
   dmesg | grep -i uart
   # Check if UART driver is loaded
   lsmod | grep serial
   ```

### 10. Integration with Drone Project

Once UART is working, integrate with your controller:

```python
# In src/controller.py or new test file
from src.lidar_sensor import LidarSensor

# Initialize LiDAR instead of ultrasonic
self.lidar = LidarSensor(
    port='/dev/ttyTHS1',  # or '/dev/ttyUSB0' for USB adapter
    baudrate=115200,
    measurement_rate=100
)

# Start sensor
self.lidar.start()

# Use same API as ultrasonic sensor
altitude = self.lidar.distance
```