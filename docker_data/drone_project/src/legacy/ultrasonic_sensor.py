#!/usr/bin/env python3
"""
Threaded Distance Sensor for Jetson
Continuously measures in background and provides latest data on demand
"""

import Jetson.GPIO as GPIO
import time
import threading
from collections import deque
from datetime import datetime
import statistics

class DistanceSensor:
    """
    Thread-safe ultrasonic distance sensor with continuous background measurements

    Features:
    - Runs measurements in separate thread
    - Always has latest distance data available
    - Thread-safe access to measurements
    - Configurable measurement rate
    - Built-in filtering and averaging
    - Measurement statistics and health monitoring
    """

    def __init__(self, trigger, echo,
                 measurement_rate=20,  # Hz
                 averaging_window=5,   # Number of samples to average
                 max_distance=4.0,     # meters
                 timeout=0.05):        # seconds
        """
        Initialize the distance sensor

        Args:
            trigger: Trigger pin number (BOARD numbering)
            echo: Echo pin number (BOARD numbering)
            measurement_rate: Measurements per second (Hz)
            averaging_window: Number of recent measurements to average
            max_distance: Maximum measurable distance in meters
            timeout: Timeout for echo response in seconds
        """
        # Pin configuration
        self.trigger = trigger
        self.echo = echo

        # Measurement parameters
        self.measurement_rate = measurement_rate
        self.measurement_interval = 1.0 / measurement_rate
        self.averaging_window = averaging_window
        self.max_distance = max_distance
        self.timeout = timeout

        # Thread-safe data storage
        self._lock = threading.Lock()
        self._latest_distance = None
        self._latest_timestamp = None
        self._measurement_history = deque(maxlen=averaging_window)
        self._all_measurements = deque(maxlen=1000)  # Keep last 1000 for stats

        # Statistics
        self._total_measurements = 0
        self._failed_measurements = 0
        self._start_time = time.time()

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trigger, GPIO.LOW)

        # Thread control
        self._running = False
        self._thread = None

        # Let sensor settle
        time.sleep(0.1)

    def start(self):
        """Start the measurement thread"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._measurement_loop, daemon=True)
        self._thread.start()

        # Wait for first measurement
        timeout = time.time() + 2.0
        while self._latest_distance is None and time.time() < timeout:
            time.sleep(0.01)

    def stop(self):
        """Stop the measurement thread"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _measurement_loop(self):
        """Background thread that continuously takes measurements"""
        while self._running:
            loop_start = time.time()

            # Take measurement
            distance = self._measure_once()
            timestamp = datetime.now()

            # Update statistics
            self._total_measurements += 1

            if distance is not None and distance <= self.max_distance:
                # Valid measurement
                with self._lock:
                    self._latest_distance = distance
                    self._latest_timestamp = timestamp
                    self._measurement_history.append(distance)
                    self._all_measurements.append((timestamp, distance))
            else:
                # Failed measurement
                self._failed_measurements += 1
                # Keep the last valid measurement available

            # Maintain measurement rate
            elapsed = time.time() - loop_start
            if elapsed < self.measurement_interval:
                time.sleep(self.measurement_interval - elapsed)

    def _measure_once(self):
        """Take a single distance measurement"""
        # Clear trigger
        GPIO.output(self.trigger, GPIO.LOW)
        time.sleep(0.000002)

        # Send trigger pulse
        GPIO.output(self.trigger, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trigger, GPIO.LOW)

        # Wait for echo start
        pulse_start = None
        timeout_time = time.time() + self.timeout

        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()
            if pulse_start > timeout_time:
                return None

        # Wait for echo end
        pulse_end = None
        timeout_time = time.time() + self.timeout

        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()
            if pulse_end > timeout_time:
                return None

        if pulse_start is None or pulse_end is None:
            return None

        # Calculate distance in meters
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * 34300) / 2 / 100  # cm to meters

        # Sanity check
        if distance < 0.02 or distance > self.max_distance:
            return None

        return distance

    # ===== Public Properties for Easy Access =====

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

    @property
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
            'min_distance': min(distances) if distances else None,
            'max_distance': max(distances) if distances else None,
            'std_deviation': statistics.stdev(recent_distances) if len(recent_distances) > 1 else 0,
            'success_rate': self.success_rate,
            'actual_rate_hz': self.measurement_rate_actual,
            'total_measurements': self._total_measurements,
            'failed_measurements': self._failed_measurements,
            'measurement_age': self.age
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

    def cleanup(self):
        """Clean up resources"""
        self.stop()
        GPIO.cleanup([self.trigger, self.echo])

    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
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
    print("Testing Threaded Distance Sensor")
    print("=================================\n")

    # Create and start sensor
    # Note: Using pins 7 and 11 with voltage divider on echo pin
    sensor = DistanceSensor(trigger=7, echo=11, measurement_rate=20)

    print("Starting sensor...")
    sensor.start()

    # Wait for first measurement
    if not sensor.wait_for_distance(timeout=2.0):
        print("Failed to get initial measurement!")
        sensor.cleanup()
        exit(1)

    print(f"Sensor started! Initial distance: {sensor.distance_cm:.1f} cm\n")

    try:
        # Continuous monitoring
        print("Distance readings (Ctrl+C to stop):")
        print("-" * 60)

        while True:
            # Access distance anytime - always gets latest value
            dist = sensor.distance
            dist_cm = sensor.distance_cm
            dist_avg = sensor.distance_averaged
            age = sensor.age

            if dist is not None:
                # Create visual bar
                bar_length = int(dist_cm / 5)
                bar = '█' * min(bar_length, 40)


                print(f"Distance: {dist_cm:6.1f} cm | "
                      f"Avg: {dist_avg*100:6.1f} cm | "
                      f"Age: {age:4.2f}s | "
                      f"Rate: {sensor.measurement_rate_actual:4.1f} Hz | "
                      f"[{bar:<40}]", end='\r')
            else:
                print("No valid measurement available", end='\r')

            time.sleep(0.05)  # Update display at 20Hz

    except KeyboardInterrupt:
        print("\n\n" + "-" * 60)
        print("Sensor Statistics:")
        stats = sensor.get_statistics()
        for key, value in stats.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.2f}")
            else:
                print(f"  {key}: {value}")

    finally:
        sensor.cleanup()
        GPIO.cleanup()
        print("\nCleanup complete")
