# Control Loop Timing Analysis

## Previous Issues with 0.1s (10Hz) Loop

Running the main control loop at 10Hz (100ms intervals) was too slow for drone control:

1. **Altitude Changes**: In 100ms, a drone can rise/fall significantly
   - At 2 m/s vertical velocity: 20cm change per loop iteration
   - This leads to poor altitude tracking and oscillations

2. **Response Delay**: 
   - Minimum reaction time to disturbances: 100ms
   - Total control latency could exceed 200ms with processing

3. **Sensor Underutilization**:
   - Ultrasonic sensor capable of 20+ Hz
   - Wasting 90% of available sensor data

## Improved Timing Architecture

### Main Loop: 50Hz (20ms)
- Critical flight control runs every iteration
- Altitude measurement and control
- Safety checks and limits

### Message Handling: 10Hz (100ms)  
- Command processing doesn't need high frequency
- Reduces CPU load from JSON parsing

### Battery Monitoring: 0.5Hz (2s)
- Battery voltage changes slowly
- No need for frequent checks

### Performance Logging: 0.2Hz (5s)
- Timing statistics logged periodically
- Prevents log spam

## Benefits of Higher Frequency

1. **Better Stability**:
   - 5x faster response to altitude changes
   - Reduced oscillations and overshoot
   - Smoother throttle adjustments

2. **Improved Disturbance Rejection**:
   - Faster recovery from wind gusts
   - Better handling of weight shifts

3. **Sensor Data Utilization**:
   - Sensor runs at 100Hz (2x oversampling)
   - Averaging reduces noise while maintaining responsiveness

4. **PID Performance**:
   - Derivative term more accurate with higher sample rate
   - Better velocity estimation from position changes

## Configuration

Control loop frequency can be adjusted in `src/altitude_config.py`:

```python
CONTROL = {
    'update_rate': 50,  # Hz - can increase to 100Hz if needed
    ...
}
```

## Monitoring Loop Performance

The controller logs timing statistics every 5 seconds:
```
Loop timing - Avg: 18.5ms (54.1Hz), Min: 15.2ms, Max: 22.1ms, Target: 20.0ms (50Hz)
```

If you see "Control loop overrun!" warnings, it means the loop is taking longer than the target time. This could indicate:
- CPU overload
- Blocking I/O operations
- Need to optimize code or reduce frequency

## Recommended Frequencies for Different Platforms

- **Jetson Nano**: 50-100Hz (plenty of CPU power)
- **Raspberry Pi 4**: 50Hz (good balance)
- **Raspberry Pi Zero**: 20-30Hz (limited CPU)

Higher frequencies provide better control but require more CPU. The sweet spot for most drones is 50-100Hz for altitude control.