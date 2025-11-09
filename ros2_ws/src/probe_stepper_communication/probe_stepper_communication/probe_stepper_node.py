# "drive" in the comments refers to stepper motor drive, specifically the LMDCE421

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from servo_interfaces.action import Servo

from pymodbus.client import ModbusTcpClient

# global variables
STEPS_PER_REVOLUTION = 51200 # default steps per revolution

# drive function registers (in use)
MOVE_ABSOLUTE_REGISTER = 0x0043
ERROR_FLAG_REGISTER = 0x001F
ERROR_VARIABLE_REGISTER = 0x0021
DRIVE_ENABLE_REGISTER = 0x001C


MOTOR_IP = '192.168.33.1'  # default drive IP
DRIVE_PORT = 502  # Standard MODBUS TCP port

class SwitchCaseOperation:

    def __init__(self, logger):
        self.logger_ = logger
        
    def operation_0(self):
        self.logger_.info("[SHUTDOWN] [start]")

        # disconnect from the drive 
        self.client.close()

        # do: disconnect power to drive

        self.logger_.info("[SHUTDOWN] [success]")
        return True

    def operation_1(self):
        self.logger_.info("[BOOT] [start]")

        # establish connection to drive
        self.client = ModbusTcpClient(DRIVE_IP, port=DRIVE_PORT)
        self.client.connect()
        if not(self.client.connected):
            self.logger_.info("[BOOT] [error]")
            return False

        # drive disable (ensure no motion through interrupted action requests)
        response = self.client.write_registers(DRIVE_ENABLE_REGISTER, [0])
        if response.isError():
            self.logger_.info("[BOOT] [drive disable] [error]")
            return False
        else:
            self.logger_.info("[BOOT] [drive disable] [success]")

        # do checks
            # do drive checks
        response = self.client.read_holding_registers(ERROR_FLAG_REGISTER)
        if response.isError():
            self.logger_.info("[BOOT] [error flag read] [error]")
            response = self.client.read_holding_registers(ERROR_VARIABLE_REGISTER)
            if response.isError():
                self.logger_.info("[BOOT] [error variable read] [error]")
                return False
            else:
                self.logger_info(f"[BOOT] [error variable read] [success] {response.registers}")
            return False
        else:
            self.logger_.info(f"[BOOT] [error flag read] [success] {response.registers}")

            # do: do log checks

        #drive enable
        response = self.client.write_registers(DRIVE_ENABLE_REGISTER, [1])
        if response.isError():
            self.logger_.info("[BOOT] [drive enable] [error]")
            return False
        else:
            self.logger_.info("[BOOT] [drive enable] [success]")

        self.logger_.info("[BOOT] [success]")

        return True

    def operation_2(self):
        self.logger_.info("[BOOT,TEST] [start]")

        if self.operation_1():
            if self.operation_5():
                self.logger_.info("[BOOT,TEST] [success]")
                return True

        self.logger_.info("[BOOT,TEST] [error]")

        return False

    def operation_3(self):
        self.logger_.info("[BOOT,TEST,REACH] [start]")

        if self.operation_1():
            if self.operation_5():
                if self.operation_6():
                    self.logger_.info("[BOOT,TEST,REACH] [success]")
                    return True

        self.logger_.info("[BOOT,TEST,REACH] [success]")
        return False

    def operation_4(self):
        self.logger_.info("[BOOT,TEST,REACH,READ] [start]")

        if self.operation_1():
            if self.operation_5():
                if self.operation_6():
                    if self.operation_7():
                        self.logger_.info("[BOOT,TEST,REACH,READ] [success]")
                        return True
        
        self.logger_.info("[BOOT,TEST,REACH,READ] [error]")
        return False

    def operation_5(self):
        self.logger_.info("[TEST] [start]")

        self.logger_.info("[TEST] [success]")
        return True

    def operation_6(self):
        self.logger_.info("[REACH] [start]")
        high = (STEPS_PER_REVOLUTION >> 16) & 0xFFFF
        low = STEPS_PER_REVOLUTION & 0xFFFF
        response = self.client.write_registers(MOVE_ABSOLUTE_REGISTER, [low, high])
        if response.isError():
            self.logger_.info("[REACH] [error]")
            return False
        else:
            self.logger_.info("[REACH] [success]")
        return True

    def operation_7(self):
        self.logger_.info("[READ] [start]")

        # during read operation the drive essentially does nothing. The thermal probe makes readings at this point.
        # this operation is just to publish the state of the drive which is in "READ operation"

        self.logger_.info("[READ] [success]")
        return True

    def operation_8(self):
        self.logger_.info("[RETRACT] [start]")

        high = ((-1 * STEPS_PER_REVOLUTION) >> 16) & 0xFFFF
        low = (-1 * STEPS_PER_REVOLUTION) & 0xFFFF
        response = self.client.write_registers(MOVE_ABSOLUTE_REGISTER, [low, high])
        if response.isError():
            self.logger_.info("[RETRACT] [error]")
            return False
        else:
            self.logger_.info("[RETRACT] [success]")

        return True

    def do_operation(self, operation_ID):
        operation_name = "operation_" + str(operation_ID)
        operation = getattr(self, operation_name)
        return operation()


class ProbeStepperNode(Node):

    def __init__(self):
        super().__init__("probe_stepper")

        # do: servo parameters. global variables must be changed to node parameters where needed.
        
        # do: servo_feedback topic

        #servo_action_server
        self.servo_operation = ActionServer(self, Servo, 'probe_stepper_operation', self.servo_callback)

        self.switchCaseOperation = SwitchCaseOperation(self.get_logger()) #instantiate switchCaseOperation once for servo_callback to use multiple times

    def servo_callback(self, goal_handle):
        
        operation_ID = goal_handle.request.operation

        self.get_logger().info(f"servo: operation[{operation_ID}]")

        # call operations based on received operation ID
        result = Servo.Result()
        result.success = self.switchCaseOperation.do_operation(operation_ID)
        goal_handle.succeed()
        return result

def main(args=None): 
    rclpy.init(args=args)
    node = ProbeStepperNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()