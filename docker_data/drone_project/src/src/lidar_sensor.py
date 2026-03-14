#!/usr/bin/env python3
"""
Threaded TF-Luna LiDAR Sensor for Jetson
Continuously reads distance data via UART and provides latest measurements on demand
Compatible with TF-Luna LiDAR sensor using UART protocol
"""

import serial
import time
import threading
from collections import deque
from datetime import datetime
import statistics
import struct

class LidarSensor:
    """
    Thread-safe TF-Luna LiDAR distance sensor with continuous background measurements
    
    Features:
    - UART communication with TF-Luna sensor
    - Runs measurements in separate thread
    - Always has latest distance data available
    - Thread-safe access to measurements
    - Built-in filtering and averaging
    - Measurement statistics and health monitoring
    - Compatible API with DistanceSensor class
    """
    
    # TF-Luna protocol constants
    FRAME_HEADER = 0x59
    FRAME_LENGTH = 9
    
    def __init__(self, port='/dev/ttyTHS1', 
                 baudrate=115200,
                 measurement_rate=200,  # Hz (TF-Luna can do up to 250Hz)
                 averaging_window=5,    # Number of samples to average
                 max_distance=8.0,      # meters (TF-Luna range: 0.2-8m)
                 min_distance=0.01,      # meters
                 signal_strength_threshold=100,  # Minimum signal strength
                 buffer_size=1024,      # Maximum buffer size
                 max_frame_age=1.0):    # Maximum age for partial frames
        """
        Initialize the TF-Luna LiDAR sensor
        
        Args:
            port: Serial port for UART communication (default: Jetson Nano UART)
            baudrate: Baud rate (TF-Luna default: 115200)
            measurement_rate: Desired measurements per second (Hz)
            averaging_window: Number of recent measurements to average
            max_distance: Maximum measurable distance in meters
            min_distance: Minimum measurable distance in meters
        """
        # Serial configuration
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
        # Measurement parameters
        self.measurement_rate = measurement_rate
        self.measurement_interval = 1.0 / measurement_rate
        self.averaging_window = averaging_window
        self.max_distance = max_distance
        self.min_distance = min_distance
        
        # Thread-safe data storage
        self._lock = threading.Lock()
        self._latest_distance = None
        self._latest_timestamp = None
        self._latest_strength = None  # Signal strength from TF-Luna
        self._latest_temperature = None  # Internal temperature
        self._measurement_history = deque(maxlen=averaging_window)
        self._all_measurements = deque(maxlen=1000)  # Keep last 1000 for stats
        
        # Statistics
        self._total_measurements = 0
        self._failed_measurements = 0
        self._checksum_errors = 0
        self._start_time = time.time()
        
        # Thread control
        self._running = False
        self._thread = None
        self._last_frame_time = None
        
        # Frame buffer for parsing
        self._buffer = bytearray()
        self._buffer_size = buffer_size
        self._max_frame_age = max_frame_age
        self._signal_strength_threshold = signal_strength_threshold
        
        # Performance monitoring
        self._last_measurement_time = time.time()
        self._frames_processed = 0
        self._buffer_overflows = 0
        self._sync_recoveries = 0
        self._serial_errors = 0
        self._consecutive_errors = 0
        
    def start(self):
        """Start the measurement thread and initialize serial connection"""
        if self._running:
            return
            
        try:
            # Open serial port
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Clear any existing data
            self.serial.reset_input_buffer()
            
            # Start measurement thread
            self._running = True
            self._thread = threading.Thread(target=self._measurement_loop, daemon=True)
            self._thread.start()
            
            # Wait for first measurement
            timeout = time.time() + 2.0
            while self._latest_distance is None and time.time() < timeout:
                time.sleep(0.01)
                
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {self.port}: {e}")
    
    def stop(self):
        """Stop the measurement thread and close serial connection"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _reconnect_serial(self):
        """Attempt to reconnect serial port"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            time.sleep(0.5)  # Brief pause before reconnecting
            
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            self.serial.reset_input_buffer()
            self._buffer.clear()
            print(f"Successfully reconnected to {self.port}")
            
        except Exception as e:
            print(f"Failed to reconnect serial port: {e}")
            time.sleep(1.0)
    
    def _measurement_loop(self):
        """Background thread that continuously reads measurements from TF-Luna"""
        error_backoff = 0.001
        
        while self._running:
            try:
                # Check for serial connection
                if not self.serial or not self.serial.is_open:
                    self._reconnect_serial()
                    continue
                
                # Read available data
                bytes_available = self.serial.in_waiting
                if bytes_available > 0:
                    # Limit read size to prevent blocking
                    read_size = min(bytes_available, 256)
                    data = self.serial.read(read_size)
                    
                    # Check buffer size limit
                    if len(self._buffer) + len(data) > self._buffer_size:
                        # Buffer overflow - clear old data
                        self._buffer_overflows += 1
                        self._buffer.clear()
                    
                    self._buffer.extend(data)
                    
                    # Process buffer for complete frames
                    frames_found = self._process_buffer()
                    
                    # Adaptive sleep based on data rate
                    if frames_found > 0:
                        time.sleep(0.0001)  # Minimal sleep when actively processing
                    else:
                        time.sleep(0.001)  # Standard sleep
                    
                    # Reset error tracking on success
                    self._consecutive_errors = 0
                    error_backoff = 0.001
                else:
                    # No data available
                    time.sleep(self.measurement_interval / 4)  # Sleep quarter of measurement interval
                
                # Check for stale partial frames
                if self._last_frame_time and len(self._buffer) > 0:
                    if time.time() - self._last_frame_time > self._max_frame_age:
                        # Clear stale buffer
                        self._buffer.clear()
                        self._sync_recoveries += 1
                
            except serial.SerialException as e:
                self._serial_errors += 1
                self._consecutive_errors += 1
                print(f"Serial error in measurement loop: {e}")
                
                # Exponential backoff for serial errors
                time.sleep(min(error_backoff, 1.0))
                error_backoff *= 2
                
                # Try to recover
                if self._consecutive_errors > 5:
                    self._reconnect_serial()
                    self._consecutive_errors = 0
                    
            except Exception as e:
                print(f"Error in measurement loop: {e}")
                self._failed_measurements += 1
                time.sleep(0.01)
    
    def _process_buffer(self):
        """Process buffer to extract TF-Luna data frames"""
        frames_found = 0
        buffer_view = memoryview(self._buffer)
        
        # Find all valid frames in buffer
        i = 0
        while i <= len(buffer_view) - self.FRAME_LENGTH:
            # Look for frame header using memoryview (more efficient)
            if buffer_view[i] == self.FRAME_HEADER and buffer_view[i+1] == self.FRAME_HEADER:
                # Potential frame found
                frame = buffer_view[i:i+self.FRAME_LENGTH]
                
                # Verify checksum
                checksum = sum(frame[:8]) & 0xFF
                if checksum == frame[8]:
                    # Valid frame - process it
                    if self._process_frame(bytes(frame)):
                        frames_found += 1
                        self._frames_processed += 1
                    
                    # Jump past this frame
                    i += self.FRAME_LENGTH
                    self._last_frame_time = time.time()
                else:
                    # Invalid checksum - skip first byte only
                    self._checksum_errors += 1
                    i += 1
            else:
                # No header found - skip byte
                i += 1
        
        # Remove processed data from buffer
        if i > 0:
            self._buffer = self._buffer[i:]
        
        return frames_found
    
    def _process_frame(self, frame):
        """Process a single validated TF-Luna frame"""
        try:
            # Parse frame data
            distance_raw = struct.unpack('<H', frame[2:4])[0]  # Little-endian uint16
            strength = struct.unpack('<H', frame[4:6])[0]
            temperature_raw = struct.unpack('<H', frame[6:8])[0]
            
            # Convert values
            distance = distance_raw / 100.0  # cm to meters
            temperature = temperature_raw / 8.0 - 256.0  # Convert to Celsius
            
            # Validate distance with configurable strength threshold
            if (self.min_distance <= distance <= self.max_distance and 
                strength >= self._signal_strength_threshold):
                
                # Additional validation: rate of change check
                if self._latest_distance is not None:
                    # Check for unrealistic jumps (> 10m/s)
                    time_delta = time.time() - self._last_measurement_time
                    if time_delta > 0:
                        velocity = abs(distance - self._latest_distance) / time_delta
                        if velocity > 10.0:  # 10 m/s max reasonable velocity
                            self._failed_measurements += 1
                            return False
                
                timestamp = datetime.now()
                
                with self._lock:
                    self._latest_distance = distance
                    self._latest_timestamp = timestamp
                    self._latest_strength = strength
                    self._latest_temperature = temperature
                    self._measurement_history.append(distance)
                    self._all_measurements.append((timestamp, distance))
                
                self._total_measurements += 1
                self._last_measurement_time = time.time()
                return True
            else:
                self._failed_measurements += 1
                return False
                
        except Exception as e:
            print(f"Error processing frame: {e}")
            self._failed_measurements += 1
            return False
    
    # ===== Public Properties for Easy Access (Same API as DistanceSensor) =====
    
    @property
    def distance(self):
        """Get the latest distance measurement in meters"""
        with self._lock:
            return self._latest_distance
    
    @property
    def distance_cm(self):
        """Get the latest distance measurement in centimeters"""
        dist = self.distance
        return dist * 100 if dist is not None else None
    
    @property
    def distance_mm(self):
        """Get the latest distance measurement in millimeters"""
        dist = self.distance
        return dist * 1000 if dist is not None else None
    
    @property
    def distance_averaged(self):
        """Get averaged distance over the window (more stable)"""
        with self._lock:
            if not self._measurement_history:
                return None
            return statistics.mean(self._measurement_history)
    
    @property
    def distance_median(self):
        """Get median distance over the window (filters outliers)"""
        with self._lock:
            if not self._measurement_history:
                return None
            return statistics.median(self._measurement_history)
    
    @property
    def signal_strength(self):
        """Get the latest signal strength value (TF-Luna specific)"""
        with self._lock:
            return self._latest_strength
    
    @property
    def temperature(self):
        """Get the internal temperature in Celsius (TF-Luna specific)"""
        with self._lock:
            return self._latest_temperature
    
    @property
    def timestamp(self):
        """Get timestamp of the latest measurement"""
        with self._lock:
            return self._latest_timestamp
    
    @property
    def age(self):
        """Get age of the latest measurement in seconds"""
        with self._lock:
            if self._latest_timestamp is None:
                return None
            return (datetime.now() - self._latest_timestamp).total_seconds()
    
    def is_fresh(self, max_age=0.1):
        """Check if the latest measurement is fresh (< max_age seconds old)"""
        age = self.age
        return age is not None and age < max_age
    
    @property
    def success_rate(self):
        """Get measurement success rate as percentage"""
        if self._total_measurements == 0:
            return 0.0
        return ((self._total_measurements - self._failed_measurements) / 
                self._total_measurements * 100)
    
    @property
    def measurement_rate_actual(self):
        """Get actual measurement rate in Hz"""
        runtime = time.time() - self._start_time
        return self._total_measurements / runtime if runtime > 0 else 0
    
    # ===== Utility Methods =====
    
    def get_statistics(self):
        """Get detailed statistics about sensor performance"""
        with self._lock:
            if not self._all_measurements:
                return None
            
            distances = [d for _, d in self._all_measurements]
            recent_distances = list(self._measurement_history)
        
        return {
            'current_distance': self.distance,
            'averaged_distance': self.distance_averaged,
            'median_distance': self.distance_median,
            'signal_strength': self.signal_strength,
            'temperature': self.temperature,
            'min_distance': min(distances) if distances else None,
            'max_distance': max(distances) if distances else None,
            'std_deviation': statistics.stdev(recent_distances) if len(recent_distances) > 1 else 0,
            'success_rate': self.success_rate,
            'checksum_errors': self._checksum_errors,
            'actual_rate_hz': self.measurement_rate_actual,
            'total_measurements': self._total_measurements,
            'failed_measurements': self._failed_measurements,
            'measurement_age': self.age,
            'frames_processed': self._frames_processed,
            'buffer_overflows': self._buffer_overflows,
            'sync_recoveries': self._sync_recoveries,
            'serial_errors': self._serial_errors,
            'signal_threshold': self._signal_strength_threshold
        }
    
    def wait_for_distance(self, timeout=5.0):
        """Wait for a valid distance measurement"""
        start = time.time()
        while self.distance is None:
            if time.time() - start > timeout:
                return False
            time.sleep(0.01)
        return True
    
    def in_range(self, min_distance=0, max_distance=None):
        """Check if current distance is within specified range"""
        dist = self.distance
        if dist is None:
            return False
        if max_distance is None:
            max_distance = self.max_distance
        return min_distance <= dist <= max_distance
    
    def set_signal_threshold(self, threshold):
        """Set minimum signal strength threshold for valid measurements"""
        self._signal_strength_threshold = threshold
    
    def set_distance_limits(self, min_distance=None, max_distance=None):
        """Update distance limits for validation"""
        if min_distance is not None:
            self.min_distance = min_distance
        if max_distance is not None:
            self.max_distance = max_distance
    
    def set_averaging_window(self, window_size):
        """Change the averaging window size"""
        with self._lock:
            self.averaging_window = window_size
            # Resize the measurement history deque
            old_measurements = list(self._measurement_history)
            self._measurement_history = deque(old_measurements[-window_size:], maxlen=window_size)
    
    def clear_buffers(self):
        """Clear all buffers and reset statistics"""
        with self._lock:
            self._buffer.clear()
            self._measurement_history.clear()
            self._checksum_errors = 0
            self._buffer_overflows = 0
            self._sync_recoveries = 0
            self._serial_errors = 0
    
    def get_buffer_status(self):
        """Get current buffer status"""
        return {
            'buffer_size': len(self._buffer),
            'buffer_limit': self._buffer_size,
            'buffer_usage_percent': (len(self._buffer) / self._buffer_size) * 100,
            'overflows': self._buffer_overflows,
            'sync_recoveries': self._sync_recoveries
        }
    
    def set_output_rate(self, rate_hz):  # noqa: F841
        """
        Set TF-Luna output rate (requires sending command to sensor)
        Supported rates: 1, 2, 5, 10, 20, 25, 50, 100, 125, 200, 250 Hz
        """
        # This would require implementing TF-Luna command protocol
        # For now, we rely on the default rate
        pass
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
    
    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self
    
    def __exit__(self, _exc_type, _exc_val, _exc_tb):
        """Context manager exit"""
        self.cleanup()
    
    def __del__(self):
        """Destructor"""
        try:
            self.cleanup()
        except:
            pass


# Example usage and testing
if __name__ == "__main__":
    print("Testing TF-Luna LiDAR Sensor")
    print("============================\n")
    
    # Create and start sensor
    # Note: Using /dev/ttyTHS1 for Jetson Nano UART
    # Make sure TF-Luna is connected with proper voltage levels (3.3V)
    sensor = LidarSensor(port='/dev/ttyTHS1', baudrate=115200, measurement_rate=100)
    
    print("Starting sensor...")
    try:
        sensor.start()
    except RuntimeError as e:
        print(f"Failed to start sensor: {e}")
        print("\nMake sure:")
        print("1. TF-Luna is connected to UART pins (TX to RX, RX to TX)")
        print("2. Power supply is 5V (signal levels should be 3.3V)")
        print("3. Serial port is correct (/dev/ttyTHS1 for Jetson Nano)")
        print("4. User has permission to access serial port (add to dialout group)")
        exit(1)
    
    # Wait for first measurement
    if not sensor.wait_for_distance(timeout=2.0):
        print("Failed to get initial measurement!")
        sensor.cleanup()
        exit(1)
    
    print(f"Sensor started! Initial distance: {sensor.distance_cm:.1f} cm")
    print(f"Signal strength: {sensor.signal_strength}")
    print(f"Temperature: {sensor.temperature:.1f}°C\n")
    
    try:
        # Continuous monitoring
        print("Distance readings (Ctrl+C to stop):")
        print("-" * 80)
        
        while True:
            # Access distance anytime - always gets latest value
            dist = sensor.distance
            dist_cm = sensor.distance_cm
            dist_avg = sensor.distance_averaged
            strength = sensor.signal_strength
            temp = sensor.temperature
            age = sensor.age
            
            if dist is not None:
                # Create visual bar
                bar_length = int(dist_cm / 10)
                bar = '█' * min(bar_length, 40)
                
                print(f"Distance: {dist_cm:6.1f} cm | "
                      f"Avg: {dist_avg*100:6.1f} cm | "
                      f"Strength: {strength:4d} | "
                      f"Temp: {temp:4.1f}°C | "
                      f"Age: {age:4.3f}s | "
                      f"[{bar:<40}]", end='\r')
            else:
                print("No valid measurement available" + " " * 40, end='\r')
            
            time.sleep(0.05)  # Update display at 20Hz
            
    except KeyboardInterrupt:
        print("\n\n" + "-" * 80)
        print("Sensor Statistics:")
        stats = sensor.get_statistics()
        for key, value in stats.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.2f}")
            else:
                print(f"  {key}: {value}")
    
    finally:
        sensor.cleanup()
        print("\nCleanup complete")