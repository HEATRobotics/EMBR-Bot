# BM680

It is a breakout for the 4-in-1 BME680 gas sensor from Bosch. For environment gas sensing it measures the resistance o the gas layer, indicating the amount of volatile components in the air.
For configuration, it uses Arduino library, which is kind of based in C/C++. However, it is possible to use python to configure the sensor. For more information check section below.

## Properties:
- Humidity
- Temperature 
- Pressure
- VOC gas change detection (gas resistance)

## Communication Interface
The sensor uses two potocols to communicate:
- I2C: half duplex communication protocols used for communication between chips. 
- SPI: full-duplex protocol facilitate communication between microcontroller and peripheral integrated circuits. 

## Integrating with Cloud Systems

- Based on the cloud provider we can use Python SDK (Software Development kits) for interacting  with the sensor API. 
- There is a bosche's library that is microcontroller agnostic [link](https://github.com/boschsensortec/BME68x_SensorAPI/tree/master), which uses dependency injection to pass necessary dependencies
via a const "const struct bme68x_dev *dev" found in the bme68x_defs.h file on the sensor API. Basically we need to:
  1. I2C read function
  2. I2C write function
  3. Delay function
  4. Set bme68x_intf to either I2C or SPI

## Installing Arduino library

 1. Install Arduino iDE
 2. Install BME680 library[link](https://github.com/Zanduino/BME680/wiki/Installation)
 3. In the IDE menu, click Sketch > Include Library > Add .ZIP Library
 4. Choose BME680master library
 5. If successful, you will get "library installed" on the output session.  

### Using Python to interface BME680 

It is possible to use Adafruit's circuitpython or SMBus library for configuring the sensor. Here it is a step by step on how to setup using Raspberry Pi or similar SBC:

Using adafruit circuitpython:

 1. Install Adafruit CircuitPython BM680
 2. Install libraries to use hardware
	> Before continuing make sure your board's lib folder or root filesystem has the adafruit_bme680.mpy, and adafruit_bus_device files.
 3. Install Adafruit_blinka library, which provides circuitpython support for python.

For testing purpose, to initialize the I2C connection with the sensor use the following code:

```
import board
import adafruit_bme680
i2c = board.I2C()
sensor = adafruit_bme680.Adafruit_BME680_I2C(i2c)
```

To print the data you can run:

```
print('Temperature: {} degrees C'.format(sensor.temperature))
print('Gas: {} ohms'.format(sensor.gas))
print('Humidity: {}%'.format(sensor.humidity))
print('Pressure: {}hPa'.format(sensor.pressure))
```

Source: Adafruit Library [link](https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/python-circuitpython)

## Documentation

### For BME680 library

Check out the [wiki](https://github.com/Zanduino/BME680/wiki) for additional information.

### For hardware assembly 

Check out the [link](https://learn.sparkfun.com/tutorials/sparkfun-environmental-sensor-breakout---bme68x-qwiic-hookup-guide#bme680-arduino-library) for SparkFun Environmental Sensor. 
