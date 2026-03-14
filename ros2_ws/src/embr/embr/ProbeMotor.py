import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32
from msg_interface.msg import ProbeMotorFeedback
from embr.sensors import create_sensor
from rcl_interfaces.msg import SetParametersResult

class ProbeMotor(Node):

    def __init__(self):
        super().__init__("probe_motor_server")
        
        self._log = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        
        # Create probeMotor interface
        try:
            self.probeMotor = create_sensor('probeMotor', config_path)
            mode_type = 'simulated' if isinstance(self.probeMotor.__class__.__name__, str) and 'Sim' in self.probeMotor.__class__.__name__ else 'real'
            self._log.info(f'probe motor initialized in {mode_type} mode (using {mode_type} probe motor)')
            self.probeMotor.start()
        except Exception as e:
            self._log.error(f'Failed to initialize probe motor: {e}')

        config = self.probeMotor.CONFIG
        
        self.declare_parameter('motion_range', config["motion_range"]['value'])
        self.declare_parameter('max_velocity', config['max_velocity']['value'])
        self.declare_parameter('acceleration', config['acceleration']['value'])

        self.add_on_set_parameters_callback(self.parameter_callback)

        # move probe topic
        self.moveTopic = self.create_subscription(Int32, 'move_probe', self.move_callback ,10, callback_group=self.callback_group)

        # post-move probe feedback topic 
        self.feedbackTopic = self.create_publisher(ProbeMotorFeedback, 'probe_motor_feedback',10)

    def parameter_callback(self, params):
        
        for param in params:
            try:                
                self.probeMotor.setConfigSetting(param.name, param.value)
            except Exception as e:
                return SetParametersResult(successful=False, reason=str(e))
    
        return SetParametersResult(successful=True)
            
    async def move_callback(self, received_msg : Int32) -> None:
        """
        Callback that handles move probe publishes. Commands the probe motor to move via its interface. 
        Publishes the feedback of probe motor after a command.
        
        Args:
            received_msg(Int32): The position to move the probe to in cm.
        """
        response_msg = ProbeMotorFeedback()
        move_position = received_msg.data
        
        try:
            originalPosition = self.probeMotor.readPosition()

            if originalPosition == move_position:
                self._log.info(f"probe already at position {originalPosition}")
                response_msg.has_moved = False
                response_msg.position = originalPosition

            else:
                await self.probeMotor.moveAbsolute(move_position)
                newPosition = self.probeMotor.readPosition()

                response_msg.position = newPosition
                response_msg.has_moved = newPosition != originalPosition

                self._log.info(f"probe moved {abs((originalPosition - newPosition) / (move_position - originalPosition) * 100)}%, position: {newPosition}cm")

        except (Exception, ValueError) as e:
            self._log.error(f"Communication failed with probe motor: {e}")
        
        except RuntimeError as e:
            self._log.error(e)

        self.feedbackTopic.publish(response_msg)

    def shutdown(self) -> None:
        """
        Terminate the connection and interface with the probe motor.
        """
        self.probeMotor.stop()



def main(args=None):
    rclpy.init(args=args)
    probeMotor_node = ProbeMotor()

    executor = MultiThreadedExecutor()
    executor.add_node(probeMotor_node)
    try:
        executor.spin()

    except KeyboardInterrupt as e:
        pass
        
    finally:
        probeMotor_node.shutdown() # close connection to probe motor before destroying node
        probeMotor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()