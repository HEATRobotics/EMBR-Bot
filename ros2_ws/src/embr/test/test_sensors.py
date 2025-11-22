"""Tests for sensor implementations."""

import pytest
import time
from embr.sensors import (
    SensorConfig,
    create_sensor,
    SimTemperatureSensor,
    SimCubeSensor,
    SimThermalSensor,
    SimMavlinkConnection,
)


class TestTemperatureSensor:
    """Tests for temperature sensor."""
    
    def test_simulated_sensor_basic(self):
        """Test basic simulated temperature sensor operation."""
        config = SensorConfig(mode='sim', params={'base_temp': 25.0})
        sensor = create_sensor('temperature', config)
        
        assert not sensor.is_running
        
        sensor.start()
        assert sensor.is_running
        
        temp = sensor.read()
        assert isinstance(temp, float)
        assert 20.0 < temp < 30.0  # Within reasonable range
        
        sensor.stop()
        assert not sensor.is_running
    
    def test_simulated_sensor_context_manager(self):
        """Test using sensor as context manager."""
        config = SensorConfig(mode='sim')
        sensor = create_sensor('temperature', config)
        
        with sensor:
            assert sensor.is_running
            temp = sensor.read()
            assert isinstance(temp, float)
        
        assert not sensor.is_running
    
    def test_simulated_sensor_variations(self):
        """Test temperature variations over time."""
        config = SensorConfig(mode='sim', params={'base_temp': 20.0, 'variation': 5.0})
        sensor = create_sensor('temperature', config)
        
        with sensor:
            readings = [sensor.read() for _ in range(10)]
            
            # Should have some variation
            assert min(readings) != max(readings)
            # But all should be in expected range
            assert all(10.0 < r < 30.0 for r in readings)


class TestCubeSensor:
    """Tests for Cube Orange GPS sensor."""
    
    def test_simulated_sensor_basic(self):
        """Test basic simulated GPS operation."""
        config = SensorConfig(mode='sim')
        sensor = create_sensor('cube', config)
        
        with sensor:
            gps = sensor.read()
            assert hasattr(gps, 'lat')
            assert hasattr(gps, 'lon')
            assert hasattr(gps, 'alt')
            assert hasattr(gps, 'vel')
            
            assert isinstance(gps.lat, int)
            assert isinstance(gps.lon, int)
            assert isinstance(gps.alt, int)
            assert isinstance(gps.vel, float)
    
    def test_simulated_sensor_movement(self):
        """Test GPS movement patterns."""
        config = SensorConfig(
            mode='sim',
            params={'pattern': 'circle', 'velocity': 10.0}
        )
        sensor = create_sensor('cube', config)
        
        with sensor:
            positions = [sensor.read() for _ in range(5)]
            
            # Positions should change over time
            lats = [p.lat for p in positions]
            assert len(set(lats)) > 1  # Not all the same
            
            # Velocity should match config
            assert all(abs(p.vel - 10.0) < 0.1 for p in positions)


class TestThermalSensor:
    """Tests for thermal camera sensor."""
    
    def test_simulated_sensor_basic(self):
        """Test basic simulated thermal camera."""
        config = SensorConfig(mode='sim', params={'width': 320, 'height': 240})
        sensor = create_sensor('thermal', config)
        
        with sensor:
            frame, all_boxes, largest_box = sensor.read()
            
            assert frame.shape == (240, 320, 3)
            assert isinstance(all_boxes, list)
            assert isinstance(largest_box, dict)
    
    def test_simulated_sensor_hotspot_detection(self):
        """Test hotspot detection in simulated thermal camera."""
        config = SensorConfig(
            mode='sim',
            params={'hotspot_count': 2, 'moving': False}
        )
        sensor = create_sensor('thermal', config)
        
        with sensor:
            frame, all_boxes, largest_box = sensor.read()
            
            # Should detect hotspots
            assert len(all_boxes) > 0
            assert 'x_center' in largest_box
            assert 'y_center' in largest_box
            assert 'angle_degrees' in largest_box


class TestMavlinkConnection:
    """Tests for MAVLink connection."""
    
    def test_simulated_connection_basic(self):
        """Test basic simulated MAVLink connection."""
        config = SensorConfig(mode='sim')
        conn = create_sensor('mavlink', config)
        
        with conn:
            # Send some data
            conn.send_temperature(25.5)
            conn.send_gps(377490000, -1224194000, 100000, 500)
            
            # Check sent messages
            assert len(conn.sent_messages) == 2
            assert conn.sent_messages[0]['type'] == 'temperature'
            assert conn.sent_messages[0]['value'] == 25.5
            assert conn.sent_messages[1]['type'] == 'gps'
    
    def test_simulated_connection_receive(self):
        """Test receiving messages from simulated connection."""
        config = SensorConfig(mode='sim')
        conn = create_sensor('mavlink', config)
        
        with conn:
            # Inject a message
            test_msg = {'type': 'HEARTBEAT', 'data': 'test'}
            conn.inject_message(test_msg)
            
            # Read it back
            msg = conn.read()
            assert msg == test_msg


# Fixtures for integration testing

@pytest.fixture
def sim_temperature_sensor():
    """Fixture providing a simulated temperature sensor."""
    config = SensorConfig(mode='sim', params={'base_temp': 22.0})
    sensor = create_sensor('temperature', config)
    sensor.start()
    yield sensor
    sensor.stop()


@pytest.fixture
def sim_cube_sensor():
    """Fixture providing a simulated Cube sensor."""
    config = SensorConfig(mode='sim', params={'pattern': 'hover'})
    sensor = create_sensor('cube', config)
    sensor.start()
    yield sensor
    sensor.stop()


@pytest.fixture
def sim_mavlink_connection():
    """Fixture providing a simulated MAVLink connection."""
    config = SensorConfig(mode='sim')
    conn = create_sensor('mavlink', config)
    conn.start()
    yield conn
    conn.stop()


def test_integration_temperature_to_mavlink(sim_temperature_sensor, sim_mavlink_connection):
    """Test integration between temperature sensor and MAVLink."""
    temp = sim_temperature_sensor.read()
    sim_mavlink_connection.send_temperature(temp)
    
    assert len(sim_mavlink_connection.sent_messages) == 1
    assert sim_mavlink_connection.sent_messages[0]['value'] == temp


def test_integration_cube_to_mavlink(sim_cube_sensor, sim_mavlink_connection):
    """Test integration between Cube sensor and MAVLink."""
    gps = sim_cube_sensor.read()
    sim_mavlink_connection.send_gps(gps.lat, gps.lon, gps.alt, int(gps.vel * 100))
    
    assert len(sim_mavlink_connection.sent_messages) == 1
    msg = sim_mavlink_connection.sent_messages[0]
    assert msg['lat'] == gps.lat
    assert msg['lon'] == gps.lon
