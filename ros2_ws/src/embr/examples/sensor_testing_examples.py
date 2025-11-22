"""
Example: Using sensor abstraction for testing
"""

from embr.sensors import create_sensor, SensorConfig

def example_basic_usage():
    """Basic usage of simulated temperature sensor."""
    # Create a simulated sensor
    config = SensorConfig(
        mode='sim',
        params={'base_temp': 25.0, 'variation': 3.0}
    )
    sensor = create_sensor('temperature', config)
    
    # Use context manager (recommended)
    with sensor:
        for i in range(5):
            temp = sensor.read()
            print(f"Reading {i+1}: {temp:.2f}°C")


def example_manual_lifecycle():
    """Manual sensor lifecycle management."""
    config = SensorConfig(mode='sim')
    sensor = create_sensor('temperature', config)
    
    try:
        sensor.start()
        temp = sensor.read()
        print(f"Temperature: {temp:.2f}°C")
    finally:
        sensor.stop()


def example_cube_patterns():
    """Different GPS movement patterns."""
    patterns = ['circle', 'line', 'hover']
    
    for pattern in patterns:
        config = SensorConfig(
            mode='sim',
            params={'pattern': pattern, 'velocity': 10.0}
        )
        sensor = create_sensor('cube', config)
        
        with sensor:
            print(f"\n{pattern.upper()} pattern:")
            for i in range(3):
                gps = sensor.read()
                print(f"  Position: ({gps.lat/1e7:.6f}, {gps.lon/1e7:.6f}), "
                      f"Alt: {gps.alt/1000:.1f}m, Vel: {gps.vel:.1f}m/s")


def example_thermal_detection():
    """Simulated thermal camera hotspot detection."""
    config = SensorConfig(
        mode='sim',
        params={'hotspot_count': 2, 'moving': True}
    )
    sensor = create_sensor('thermal', config)
    
    with sensor:
        frame, all_boxes, largest = sensor.read()
        print(f"Detected {len(all_boxes)} hotspots")
        if largest:
            print(f"Largest at: ({largest['x_center']}, {largest['y_center']})")
            print(f"Angle: {largest['angle_degrees']:.1f}°")


def example_mavlink_testing():
    """Testing MAVLink communication."""
    config = SensorConfig(mode='sim')
    mavlink = create_sensor('mavlink', config)
    
    with mavlink:
        # Send some data
        mavlink.send_temperature(23.5)
        mavlink.send_gps(377490000, -1224194000, 100000, 500)
        
        # Check what was sent
        print(f"Sent {len(mavlink.sent_messages)} messages:")
        for msg in mavlink.sent_messages:
            print(f"  {msg}")
        
        # Inject a received message
        mavlink.inject_message({'type': 'HEARTBEAT', 'data': 'test'})
        received = mavlink.read()
        print(f"Received: {received}")


def example_config_loading():
    """Load configuration from file."""
    from embr.sensors import SensorFactory
    
    # Load from file
    configs = SensorFactory.load_config('config/sensors_sim.json')
    
    # Create sensors from config
    temp_sensor = create_sensor('temperature', configs['temperature'])
    cube_sensor = create_sensor('cube', configs['cube'])
    
    with temp_sensor, cube_sensor:
        temp = temp_sensor.read()
        gps = cube_sensor.read()
        print(f"Temp: {temp:.2f}°C")
        print(f"GPS: {gps.lat}, {gps.lon}")


def example_auto_mode():
    """Auto-detect hardware availability."""
    # Will automatically use real hardware if available, sim otherwise
    config = SensorConfig(mode='auto')
    
    sensors_to_test = ['temperature', 'cube', 'thermal', 'mavlink']
    
    for sensor_type in sensors_to_test:
        try:
            sensor = create_sensor(sensor_type, config)
            sensor.start()
            
            mode = 'REAL' if 'Real' in sensor.__class__.__name__ else 'SIM'
            print(f"{sensor_type}: Using {mode} implementation")
            
            sensor.stop()
        except Exception as e:
            print(f"{sensor_type}: Error - {e}")


if __name__ == '__main__':
    print("=" * 60)
    print("EMBR Sensor Testing Examples")
    print("=" * 60)
    
    print("\n1. Basic Temperature Sensor Usage:")
    example_basic_usage()
    
    print("\n2. Cube GPS Movement Patterns:")
    example_cube_patterns()
    
    print("\n3. Thermal Camera Detection:")
    example_thermal_detection()
    
    print("\n4. MAVLink Communication Testing:")
    example_mavlink_testing()
    
    print("\n5. Configuration File Loading:")
    example_config_loading()
    
    print("\n6. Auto-detect Mode:")
    example_auto_mode()
