#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

from agv_pro_msgs.srv import (
    SetDigitalOutput,
    GetDigitalInput,
    SetLedColor,
    SetLedMode
)

class AGVIOClient(Node):
    def __init__(self):
        super().__init__('agv_io_client')

        # Create service client
        self.cli_set_io = self.create_client(SetDigitalOutput, 'set_digital_output')
        self.cli_get_io = self.create_client(GetDigitalInput, 'get_digital_input')
        self.cli_led_output = self.create_client(SetLedColor, 'set_led_color')
        self.cli_led_mode = self.create_client(SetLedMode, 'set_led_mode')

        # Wait until all services are available
        self._wait_for_services()

    def _wait_for_services(self):
        """Wait for all services to become available."""
        clients = [
            self.cli_set_io,
            self.cli_get_io,
            self.cli_led_output,
            self.cli_led_mode
        ]

        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                pass

    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return None

    # ------------------------------
    # Set digital output
    # ------------------------------
    def set_digital_output(self, pin: int, state: int) -> bool:
        """Set digital output pin state."""
        req = SetDigitalOutput.Request()
        req.pin = pin
        req.state = state

        res = self._call_service(self.cli_set_io, req)
        return res.success if res else False

    # ------------------------------
    # Get digital input
    # ------------------------------
    def get_digital_input(self, pin: int):
        """Read digital input pin state."""
        req = GetDigitalInput.Request()
        req.pin = pin

        res = self._call_service(self.cli_get_io, req)
        return res.state if (res and res.success) else None

    # ------------------------------
    # Set LED color
    # ------------------------------
    def set_led_color(self,
                      position: int,
                      brightness: int,
                      r: int,
                      g: int,
                      b: int) -> bool:
        """Set LED RGB color and brightness."""
        req = SetLedColor.Request()
        req.position = position
        req.brightness = brightness
        req.r = r
        req.g = g
        req.b = b

        res = self._call_service(self.cli_led_output, req)
        return res.success if res else False

    # ------------------------------
    # Set LED mode
    # ------------------------------
    def set_led_mode(self, mode: bool) -> bool:
        """Set LED mode (True/False)."""
        req = SetLedMode.Request()
        req.mode = mode

        res = self._call_service(self.cli_led_mode, req)
        return res.success if res else False


def main(args=None):
    rclpy.init(args=args)
    client = AGVIOClient()

    ################
    # Example usage:
    ################

    # client.set_digital_output(pin=1, state=1)
    # client.get_digital_input(pin=2)

    # client.set_led_mode(True)
    # for i in range(10):
    #     client.set_led_color(0, 100, 255, 255, 0)
    #     client.set_led_color(1, 100, 255, 255, 0)
    #     time.sleep(0.5)
    #     client.set_led_color(0, 0, 0, 0, 0)
    #     client.set_led_color(1, 0, 0, 0, 0)
    #     time.sleep(0.5)

    # client.set_led_color(0, 100, 255, 255, 0)
    # client.set_led_color(1, 100, 255, 255, 0)
    # client.set_led_mode(True)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()