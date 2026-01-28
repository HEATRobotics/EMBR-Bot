import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from msg_interface.msg import MoveProbeMotor, ProbeMotorFeedback
from embr.sensors import create_probeStepper, ProbeStepperConfig, StepperFactory


class ProbeStepper(Node):

    def __init__(self):
        super().__init__("probe_stepper_server")

        self._log = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file')
        config_path = config_file.value

        # Create probeStepper
        try:
            self.probeStepper = create_probeStepper('probeStepper', config_path)
            probeStepper_type = 'simulated' if isinstance(self.probeStepper.__class__.__name__, str) and 'Sim' in self.probeStepper.__class__.__name__ else 'real'
            self._log.info(f'probeStepper initialized in {probeStepper_type} mode (using {probeStepper_type} probeStepper)')
            if not self.probeStepper.start():
                self._log.error("Failed to start probe stepper")
        except Exception as e:
            self._log.error(f'Failed to initialize probeStepper: {e}')

        # to do : create parameters that is synchronized with ProbeStepper().CONFIG and does input checking
            # steps_per_distance
            # motion_range
            # max_velocity
            # acceleration

        # move topic
        self.moveTopic = self.create_subscription(MoveProbeMotor, 'move_probe_motor', self.move_callback ,10, callback_group=self.callback_group)

        # feedback topic    
        self.feedbackTopic = self.create_publisher(ProbeMotorFeedback, 'probe_motor_feedback',10)


    async def move_callback(self, receivedMsg) -> None:
        responseMsg = ProbeMotorFeedback()
        
        try:
            originalPosition = self.probeStepper.readPosition()    # store position before changing it
            if originalPosition == receivedMsg.move_position:
                self._log.info(f"probe already at position {originalPosition}")
                responseMsg.has_moved = False
                responseMsg.position = originalPosition

            else:
                success = await self.probeStepper.moveAbsolute(receivedMsg.move_position)

                newPosition = self.probeStepper.readPosition()
                responseMsg.position = newPosition
                responseMsg.has_moved = newPosition != originalPosition
                self._log.info(f"probe moved {abs((originalPosition - newPosition) / (receivedMsg.move_position - originalPosition) * 100)}%, position: {newPosition}cm")
        except Exception as e:
            self._log.error(f"Failed to command probe motor: {e}")

        self.feedbackTopic.publish(responseMsg)

    def shutdown(self) -> None:
        try:
            if not self.probeStepper.stop():
                self._log.error("Failed to stop probe motor")
        except Exception as e:
            self._log.error(f"Failed to shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    stepper_node = ProbeStepper()

    executor = MultiThreadedExecutor()
    executor.add_node(stepper_node)
    try:
        executor.spin()

    except KeyboardInterrupt as e:
        pass
        
    finally:
        stepperNode.shutdown()
        stepperNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()