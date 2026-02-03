import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32
from msg_interface.msg import ProbeMotorFeedback
from embr.sensors import create_sensor, ProbeMotorConfig


class ProbeMotor(Node):

    def __init__(self):
        super().__init__("probe_motor_server")

        self._log = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        

        # Create probeMotor
        try:
            self.probeMotor = create_sensor('probeMotor', config_path)
            mode_type = 'simulated' if isinstance(self.probeMotor.__class__.__name__, str) and 'Sim' in self.probeMotor.__class__.__name__ else 'real'
            self._log.info(f'probe motor initialized in {mode_type} mode (using {mode_type} probe motor)')
            if not self.probeMotor.start():
                self._log.error("Failed to start probe motor")
        except Exception as e:
            self._log.error(f'Failed to initialize probe motor: {e}')

        # to do : create parameters that is synchronized with ProbeMotor().CONFIG and does input checking
            # steps_per_distance
            # motion_range
            # max_velocity
            # acceleration

        # move topic
        self.moveTopic = self.create_subscription(Int32, 'move_probe_motor', self.move_callback ,10, callback_group=self.callback_group)

        # feedback topic    
        self.feedbackTopic = self.create_publisher(ProbeMotorFeedback, 'probe_motor_feedback',10)


    async def move_callback(self, received_msg : Int32) -> None:
        response_msg = ProbeMotorFeedback()
        move_position = received_msg.data
        
        try:

            originalPosition = self.probeMotor.readPosition()

            # 
            if originalPosition == move_position:
                self._log.info(f"probe already at position {originalPosition}")
                response_msg.has_moved = False
                response_msg.position = originalPosition

            else:
                success = await self.probeMotor.moveAbsolute(move_position)

                newPosition = self.probeMotor.readPosition()
                response_msg.position = newPosition
                response_msg.has_moved = newPosition != originalPosition
                self._log.info(f"probe moved {abs((originalPosition - newPosition) / (move_position - originalPosition) * 100)}%, position: {newPosition}cm")
        except Exception as e:
            self._log.error(f"Failed to command probe motor: {e}")

        self.feedbackTopic.publish(response_msg)

    def shutdown(self) -> None:
        try:
            if not self.probeMotor.stop():
                self._log.error("Failed to stop probe motor")
        except Exception as e:
            self._log.error(f"Failed to shutdown: {e}")


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
        probeMotor_node.shutdown()
        probeMotor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()