### to do : handle case where physical connection to motor is lost during operation

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from dataclasses import dataclass, field

import time # for time.sleep() in SimProbeMotor::move()
import random

from pymodbus.client import ModbusTcpClient

from embr.sensors import SensorConfig

@dataclass
class ProbeMotorConfig:
    mode: str
    device: Optional[str]
    baud: Optional[str] 
    params: Dict[str, Any]


class ProbeMotorBase(ABC):

    CONFIG = { # settings : value
        "steps_per_distance" : -1,   # unit in step/centi meter, translation of rotation(steps) to vertical displacement # to do: needs to be calliberated
        "motion_range" : -1,         # unit in centi meter
        "max_velocity" : -1,         # unit in step/second
        "acceleration" : -1          # unit in step/second^2
    }

    def __init__(self, config: Optional[SensorConfig] = None):
        
        
        self._running = False
        self.start()

        # initialize CONFIG settings' value
        params = config["params"]
        print("params initialized")
        if params:
            for setting, value in params.items():
                if params[setting]:
                    self.setConfigSetting(setting, value)    
                else:
                    raise ValueError(f"Invalid configuration setting")
        else:
            raise ValueError(f"Probe motor configuration is empty")


        

    @abstractmethod
    def start(self) -> bool:
        pass

    @abstractmethod
    async def moveAbsolute(self, distance) -> bool:
        # to do:
            # check distance is within motion range
        pass

    @abstractmethod
    def readPosition(self) -> int:
        pass

    @abstractmethod
    def setConfigSetting(self, setting, value) -> bool:
        # to do:
            # raise possible errors:
                # motion_range > steps_per_distance
                # all config values = 0
        pass

    @abstractmethod
    def stop(self) -> bool:
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
        return 


class RealProbeMotor(ProbeMotorBase):

    DRIVE_IP = '192.168.33.1'  # default drive IP
    DRIVE_PORT = 502  # Standard MODBUS TCP port
     
    REGISTERS = {
        "move_absolute" : {"address" : 0x0043, "length" : 2},
        "position_counter" : {"address" : 0x0057, "length" : 2},
        "max_velocity" : {"address" : 0x008B, "length" : 2},
        "acceleration" : {"address" : 0x0000, "length" : 1}
    }

    def __init__(self, config: Optional[ProbeMotorConfig] = None):
        super().__init__(config)
        self.client = None

    def _write_registers(self, register, value):
        if register["length"] == 1:
            return self.client.write_registers(register["address"], [value])
        elif register["length"] == 2:
            return self.client.write_registers(register["address"], [low,high])

    def _read_registers(self, register):
        if register["length"] == 1:
            response = self.client.read_holding_registers(register["address"])
            if not response.isError():
                return response, response.value
            else:
                return response, None
        elif register["length"] == 2:
            response = self.client.read_holding_registers(register["address"])
            if not response.isError():
                value = response.registers[0] | (response.registers[1] >> 16)
                return response, value
            else:
                return response, None

    def _updateMotorSettings(self, setting : str, value : int) -> bool:
            response = self._write_registers(self.REGISTERS["setting"], value)
            if response.isError():
                return False
        

    def start(self) -> bool:

        if self.is_running:
            return True

        #Establish connection to the probe motor
        self.client = ModbusTcpClient(self.DRIVE_IP, port=self.DRIVE_PORT)
        self.client.connect()

        if not self.client.connected:
            return False

        self.set_running()
        return True
  
    async def moveAbsolute(self, position) -> bool:

        if not self.is_running:
            return False

        positionInSteps = position * self.CONFIG["steps_per_distance"]

        # spin the motor by number of steps
        response = self._write_registers(REGISTERS["move_absolute"], positionInSteps)
        if response.isError():
            # log error
            return False
        
        return True

    def readPosition(self) -> int:
        
        if not self.is_running:
            raise Exception("probe motor is not running")
        
        response, positionInSteps = self._read_registers(REGISTERS["position_counter"])
        
        if response.isError():
            raise Exception("met with response error")
        
        positionInDistance = positionInSteps * self.CONFIG["steps_per_distance"]

        return int(positionInDistance)

    def setConfigSetting(self, setting : str, value : int) -> bool:
        if not self.CONFIG[setting]:
            return False
        
        elif not self._updateMotorSettings(setting,value):
            return False
        
        else: 
            self.CONFIG[setting] = value
            return True

    def stop(self) -> bool:

        if not self.is_running:
            raise Exception("No active connection to probe motor")

        # disconnect from the drive 
        self.client.close()
        
        self.stop_running
        return True


class SimProbeMotor(ProbeMotorBase):

    TIME_PER_DISTANCE = 0.5 # in second/distance
    POSITION = 0            # in steps
    
    def __init__(self, config: Optional[ProbeMotorConfig] = None):
        super().__init__(config)

    def start(self) -> bool:
        if self.is_running:
            return True
        self.set_running()
        return True
    
    async def moveAbsolute(self, position) -> bool:
        if not self.is_running:
            return False

        if position > self.POSITION:
            # apply a 90% probability that the probe doesn't fully extend to the provided position
            if random.random() > 0.9: 
                position = random.randint(0,position)

        # sleep to mimic the time a motor would take to complete spin
        time.sleep(abs((position - (self.POSITION/self.CONFIG["steps_per_distance"] if self.POSITION else 0)) *  self.TIME_PER_DISTANCE)) # /100 to convert mili sec to sec

        # update the position by converting received position from cm to steps
        self.POSITION = position * self.CONFIG["steps_per_distance"]
        return True

    def readPosition(self) -> int:
        return int(self.POSITION / self.CONFIG["steps_per_distance"])

    def setConfigSetting(self, setting : str, value : int) -> bool:
        if not self.CONFIG[setting]:
            return False
        
        else: 
            self.CONFIG[setting] = value
            
            return True


    def stop(self) -> bool:

        if not self.is_running:
            raise Exception("No active connection to probe motor")

        self.stop_running()

        return True
