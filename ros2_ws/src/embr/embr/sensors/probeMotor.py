### to do : handle case where physical connection to motor is lost during operation

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from dataclasses import dataclass, field

import time # for time.sleep() in SimProbeMotor::move()
import random

from pymodbus.client import ModbusTcpClient

from embr.sensors import SensorConfig
from embr.sensors import Sensor

class ProbeMotorBase(Sensor):

    CONFIG = { # settings : {value,range}
        "steps_per_distance" : {    # unit in step/centi meter, translation of 
            "value" : 51200,          # rotation(steps) to vertical displacement       
            "range" : [0,51200]
        },   
        "motion_range" : {          # unit in centi meter
            "value" : 10,
            "range" : [0,10]
        },         
        "max_velocity" : {          # unit in step/second
            "value" : 768000,
            "range" : [0, 768000]
        },         
        "acceleration" : {          # unit in step/second^2
            "value" : 1000000,
            "range" : [0,1000000]
        }          
    }

    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)
        
        # start to update the running state
        self.start()

        # retrieve all the parameters from the supplied config
        params = config.params

        # update the respective config settings using the parameters
        if params:
            for setting, value in params.items():
                if params[setting]:
                    self.setConfigSetting(setting, value)
                else:
                    raise ValueError(f"Invalid configuration setting")
        else:
            raise ValueError(f"Probe motor configuration is empty")

    @abstractmethod
    async def moveAbsolute(self, distance : int) -> None:
        """
        Spin the motor in steps equivalent to the translated vertical distance of arg-distance.
        
        Args:
            distance (int): distance in cm.
        """
        
        pass

    @abstractmethod
    def setConfigSetting(self, setting : str, value : int) -> None:
        """
        Sets the config setting value to arg-value and update the probe motor's setting if required.
        
        Args:
            setting (string): Setting label
            value (int): Setting value
        """
        pass

    @property
    def is_running(self) -> bool:
        """Check if probe motor is running."""
        # to do: override this RealProbeMotor to also check self.client.connected
        return self._running
    
    def set_running(self) -> None:
        self._running = True

    def stop_running(self) -> None:
        self._running = False

    def getConfig():
        return self.config



class RealProbeMotor(ProbeMotorBase):

    DRIVE_IP = '192.168.33.1'  # default drive IP
    DRIVE_PORT = 502  # Standard MODBUS TCP port
     
    REGISTERS = {                                   #length = number of registers
        "move_absolute"     : {"address" : 0x0043, "length" : 2},
        "position_counter"  : {"address" : 0x0057, "length" : 2},
        "max_velocity"      : {"address" : 0x008B, "length" : 2},
        "acceleration"      : {"address" : 0x0000, "length" : 2},
        "moving"            : {"address" : 0x004A, "length" : 1},
    }

    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)

    def _write_registers(self, register, value: int) -> None:
        """
        Directly communicate with the probe motor via ethernet by manipulating the probe motor 
        registers to command it, change configuration, or mode.
        
        Args:
            register (dict[str, [str: int, str: int]]): Register dictionary containing 
                                                        the register address and number 
                                                        of relavent registers
            value (int): The value to write to the respective register(s).
        """

        if register["length"] == 1:
            response = self.client.write_registers(register["address"], [value])


        elif register["length"] == 2:
            # encode int_32 into two int_16 values(one lower 16-bits, one upper 16-bits)
            high = (value >> 16) & 0xFFFF
            low = value & 0xFFFF
            response = self.client.write_registers(register["address"], [low,high])
            
        if response.isError():
            raise RuntimeError(f"Error writing {value} to register {register}")


    def _read_registers(self, register):
        if register["length"] == 1:
            response = self.client.read_holding_registers(register["address"], count = 1)

            if not response.isError():
                return response, response.registers[0]

            else:
                raise RuntimeError(f"Error reading from register {register}")

        elif register["length"] == 2:
            response = self.client.read_holding_registers(register["address"], count = 2)

            if not response.isError():
                # decode two int_16 values(one lower 16-bits, one upper 16-bits) into one int_32 value
                value = response.registers[0] | (response.registers[1] << 16)
                return response, value

            else:
                raise RuntimeError(f"Error reading from register {register}")
    

    def start(self) -> None:
        #Establish connection to the probe motor
        self.client = ModbusTcpClient(self.DRIVE_IP, port=self.DRIVE_PORT)
        self.client.connect()

        if not self.client.connected:
            raise RuntimeError("Failed to establish connection with probe motor")

        self.set_running()


    async def _waitTillMoveFinish(self) -> None:
        """
        Meant to be used by self.moveAbsolute(distance). 
        Wait till probe motor has finished moving to specified position.
        """

        # Assume (reasonably) probe motor is moving after a successful move command
        is_moving = True

        # check every half a second for the move status of the probe motor
        while(is_moving):
            time.sleep(0.5)
            response, value = self._read_registers(self.REGISTERS["moving"])
            is_moving = value
  

    async def moveAbsolute(self, position) -> None:
        if not self.is_running:
            raise Exception("probe motor is not running")

        motion_range = self.CONFIG["motion_range"]["value"]
        if not (0 <= position <= motion_range):
            raise ValueError(f"Position {position}cm is out of motion range [0, {motion_range}]")

        positionInSteps = position * self.CONFIG["steps_per_distance"]["value"]

        # spin the motor by number of steps
        self._write_registers(self.REGISTERS["move_absolute"], positionInSteps)

        await self._waitTillMoveFinish()
        return True


    def readPosition(self) -> int:
        if not self.is_running:
            raise Exception("probe motor is not running")
        
        response, positionInSteps = self._read_registers(self.REGISTERS["position_counter"])
 
        positionInDistance = positionInSteps / self.CONFIG["steps_per_distance"]["value"]

        return int(positionInDistance)

    def read(self) -> int:
        return readPosition()


    def setConfigSetting(self, setting : str, value : int) -> None:
        if setting in self.REGISTERS:
            lowerbound = self.CONFIG[setting]['range'][0]
            upperbound = self.CONFIG[setting]['range'][1]
        
            if not (lowerbound <= value <= upperbound):
                raise ValueError(f'{setting} value {value} is not in range [{lowerbound}, {upperbound}]')

            response = self._write_registers(self.REGISTERS[setting], value)
            self.CONFIG[setting]["value"] = value
        
        else: 
            if self.CONFIG[setting]["value"]:
                self.CONFIG[setting]["value"] = value

            else:
                raise ValueError(f"No such {setting} setting in config")
            

    def stop(self) -> bool:
        if not self.is_running:
            raise Exception("No active connection to probe motor")

        # disconnect from the drive 
        self.client.close()
        
        self.stop_running()


class SimProbeMotor(ProbeMotorBase):
    TIME_PER_DISTANCE = 0.5 # in second/distance
    POSITION = 0          # in steps
    
    def __init__(self, config: Optional[SensorConfig] = None):
        super().__init__(config)

    def start(self) -> None:
        if not self.is_running:
            self.set_running()

    
    async def moveAbsolute(self, position) -> bool:
        if not self.is_running:
            raise Exception("No active connection to probe motor")

        if position > self.POSITION:
            # apply a 90% probability that the probe doesn't fully extend to the provided position
            if random.random() > 0.9: 
                position = random.randint(0,position)

        # sleep to mimic the time a motor would take to complete spin
        time.sleep(abs((position 
                        - (self.POSITION/self.CONFIG["steps_per_distance"]["value"] if self.POSITION else 0)) 
                        * self.TIME_PER_DISTANCE)) # /100 to convert mili sec to sec

        # update the position by converting received position from cm to steps
        self.POSITION = position * self.CONFIG["steps_per_distance"]["value"]


    def readPosition(self) -> int:
        return int(self.POSITION / self.CONFIG["steps_per_distance"]["value"])

    def read(self) -> int:
        return readPosition()


    def setConfigSetting(self, setting : str, value : int) -> None:
        
        if not self.CONFIG[setting]:
            raise ValueError(f"No such {setting} setting in config")
        
        else: 
            lowerbound = self.CONFIG[setting]['range'][0]
            upperbound = self.CONFIG[setting]['range'][1]
        
            if not (lowerbound <= value <= upperbound):
                raise Exception(f'{setting} value {value} is not in range [{lowerbound}, {upperbound}]')

            self.CONFIG[setting]["value"] = value


    def stop(self) -> None:
        if not self.is_running:
            raise Exception("No active connection to probe motor")

        self.stop_running()