"""Tests for sensor implementations."""

import pytest
import time
import numpy as np
from embr.sensors import (
    SensorConfig,
    create_sensor,
    SimTemperatureSensor,
    SimCubeSensor,
    SimRadioConnection,
    SimThermalCameraSensor,
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
            positions = []
            for _ in range(5):
                positions.append(sensor.read())
                time.sleep(0.1)  # Small delay to allow position to change
            
            # Positions should change over time
            lats = [p.lat for p in positions]
            assert len(set(lats)) > 1  # Not all the same
            
            # Velocity should match config
            assert all(abs(p.vel - 10.0) < 0.1 for p in positions)


class TestThermalCameraSensor:
    """Tests for thermal camera sensor."""
    
    def test_simulated_sensor_basic(self):
        """Test basic simulated thermal camera operation."""
        config = SensorConfig(
            mode='sim',
            params={
                # Use 3.1R model (default) -> 160x120 resolution
                'model': '3.1R',
                'base_temp': 22.0,
                'hotspot_temp': 40.0,
                'num_hotspots': 2
            }
        )
        sensor = create_sensor('thermal', config)
        
        assert not sensor.is_running
        
        sensor.start()
        assert sensor.is_running
        
        # Read a frame
        frame = sensor.read()
        assert isinstance(frame, np.ndarray)
        assert frame.dtype == np.uint16
        assert frame.shape == (120, 160)
        
        sensor.stop()
        assert not sensor.is_running
    
    def test_simulated_sensor_temperature_range(self):
        """Test thermal camera outputs correct temperature range."""
        config = SensorConfig(
            mode='sim',
            params={
                # Use 2.5 model -> 80x60 resolution
                'model': '2.5',
                'base_temp': 22.0,
                'temp_variation': 5.0,
                'hotspot_temp': 40.0,
                'num_hotspots': 1
            }
        )
        sensor = create_sensor('thermal', config)
        
        with sensor:
            frame = sensor.read()
            
            # Convert from Kelvin*100 to Celsius
            temp_celsius = (frame / 100.0) - 273.15
            
            # Should have base temperature around 22C
            assert temp_celsius.min() > 10.0
            assert temp_celsius.max() < 50.0
            
            # Should have at least some hot pixels
            hot_pixels = np.sum(temp_celsius > 30.0)
            assert hot_pixels > 0
    
    def test_simulated_sensor_context_manager(self):
        """Test using thermal sensor as context manager."""
        config = SensorConfig(
            mode='sim',
            params={'model': '2.5'}
        )
        sensor = create_sensor('thermal', config)
        
        with sensor:
            assert sensor.is_running
            frame = sensor.read()
            assert frame.shape == (60, 80)
        
        assert not sensor.is_running
    
    def test_simulated_sensor_frame_shape(self):
        """Test get_frame_shape method."""
        # Test both model resolutions
        config_big = SensorConfig(mode='sim', params={'model': '3.1R'})
        sensor_big = create_sensor('thermal', config_big)
        assert sensor_big.get_frame_shape() == (120, 160)
        with sensor_big:
            assert sensor_big.get_frame_shape() == (120, 160)
            frame_big = sensor_big.read()
            assert frame_big.shape == sensor_big.get_frame_shape()

        config_small = SensorConfig(mode='sim', params={'model': '2.5'})
        sensor_small = create_sensor('thermal', config_small)
        assert sensor_small.get_frame_shape() == (60, 80)
        with sensor_small:
            assert sensor_small.get_frame_shape() == (60, 80)
            frame_small = sensor_small.read()
            assert frame_small.shape == sensor_small.get_frame_shape()
    
    def test_simulated_sensor_multiple_frames(self):
        """Test reading multiple frames shows temporal variation."""
        config = SensorConfig(mode='sim', params={'num_hotspots': 2})
        sensor = create_sensor('thermal', config)
        
        with sensor:
            frames = [sensor.read() for _ in range(3)]
            
            # All frames should be valid
            assert all(isinstance(f, np.ndarray) for f in frames)
            assert all(f.dtype == np.uint16 for f in frames)
            
            # Frames should vary slightly due to temporal noise
            # (though they may be very similar in sim mode)
            means = [f.mean() for f in frames]
            assert all(20000 < m < 35000 for m in means)  # Reasonable range in Kelvin*100

class TestRadioConnection:
    """Tests for Radio connection."""
    
    def test_simulated_connection_basic(self):
        """Test basic simulated Radio connection."""
        config = SensorConfig(mode='sim')
        conn = create_sensor('radio', config)
        
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
        conn = create_sensor('radio', config)
        
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
def sim_radio_connection():
    """Fixture providing a simulated Radio connection."""
    config = SensorConfig(mode='sim')
    conn = create_sensor('radio', config)
    conn.start()
    yield conn
    conn.stop()


@pytest.fixture
def sim_thermal_sensor():
    """Fixture providing a simulated thermal camera sensor."""
    config = SensorConfig(
        mode='sim',
        params={'model': '3.1R', 'num_hotspots': 2}
    )
    sensor = create_sensor('thermal', config)
    sensor.start()
    yield sensor
    sensor.stop()


def test_integration_temperature_to_radio(sim_temperature_sensor, sim_radio_connection):
    """Test integration between temperature sensor and Radio."""
    temp = sim_temperature_sensor.read()
    sim_radio_connection.send_temperature(temp)
    
    assert len(sim_radio_connection.sent_messages) == 1
    assert sim_radio_connection.sent_messages[0]['value'] == temp


def test_integration_cube_to_radio(sim_cube_sensor, sim_radio_connection):
    """Test integration between Cube sensor and Radio."""
    gps = sim_cube_sensor.read()
    sim_radio_connection.send_gps(gps.lat, gps.lon, gps.alt, int(gps.vel * 100))
    
    assert len(sim_radio_connection.sent_messages) == 1
    msg = sim_radio_connection.sent_messages[0]
    assert msg['lat'] == gps.lat
    assert msg['lon'] == gps.lon
