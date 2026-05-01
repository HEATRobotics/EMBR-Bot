import time
import threading
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from msg_interface.msg import ProbeMotorFeedback

class Nav(Node):

    def __init__(self):
        super().__init__('nav')
        self.moveProbeMotorTopic = self.create_publisher(Int32, 'move_probe', 10)
        self.probeMotorFeedbackTopic = self.create_subscription(ProbeMotorFeedback, 'probe_motor_feedback', self.probeMotorFeedback_callback, 10)
        self.probe_motor_feedback_event = threading.Event()

        self.survey_thread = threading.Thread(target=self.startSurvey)
        self.survey_thread.start()

    def probeMotorFeedback_callback(self, feedback):
        self.probe_motor_feedback_event.has_moved = feedback.has_moved
        self.probe_motor_feedback_event.position = feedback.position
        self.probe_motor_feedback_event.set()


        

    def startSurvey(self):

        # to do: initiate motion and related components

        # trigger probe to move
        # below is an example where the probe is randomly triggered to move
        while(True):
            self.get_logger().info("searching for hotspot")

            time.sleep(random.randint(10,30))
            self.get_logger().info("found a hotspot")
            self.get_logger().info("moving towards hotspot")
            time.sleep(random.randint(5,10))
            self.get_logger().info("reached hotspot")
            self.get_logger().info("extending probe")
            if self.moveProbe(10): # displace probe to a position 5cm vertically downwards
                self.get_logger().info(f"extended probe to {self.probe_motor_feedback_event.position}")
                self.get_logger().info("reading temp values")
                time.sleep(4)
                self.get_logger().info("retracting probe")
                if not self.moveProbe(0):
                    # to do: halt operation to avoid potential harm to probe
                    self.get_logger().error("Failed to retract probe")
                self.get_logger().info("retracted probe")
                
    def moveProbe(self, toPosition) -> bool:
        msg = Int32()
        msg.data = toPosition
        self.moveProbeMotorTopic.publish(msg)
        self.probe_motor_feedback_event.wait()
        self.probe_motor_feedback_event.clear()
        if not self.probe_motor_feedback_event.has_moved and ( self.probe_motor_feedback_event.position != toPosition ):
            self.get_logger().error(f"Probe failed to move position from {self.probe_motor_feedback_event.position} to {toPosition}")
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    navNode = Nav()
    try:
        rclpy.spin(navNode)

    except KeyboardInterrupt:
        pass
        
    finally:
        navNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()