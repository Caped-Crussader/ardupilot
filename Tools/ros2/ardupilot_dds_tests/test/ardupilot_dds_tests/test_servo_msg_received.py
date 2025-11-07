# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

# flake8: noqa

"""
Bring up ArduPilot SITL and check the Servo message is being published.

Checks whether servo output messages are received via /ap/servo_out topic
and validates that PWM values are within the valid range (800-2200 microseconds).

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k test_servo_msg_received

"""

import pytest
import rclpy
import rclpy.node
import threading

from launch_pytest.tools import process as process_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from ardupilot_msgs.msg import Servo

from launch_fixtures import (
    launch_sitl_copter_dds_serial,
    launch_sitl_copter_dds_udp,
)

TOPIC = "ap/servo_out"
WAIT_FOR_START_TIMEOUT = 5.0


class ServoListener(rclpy.node.Node):
    """Subscribe to Servo messages."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("servo_listener")
        self.msg_event_object = threading.Event()
        self.valid_channels_event = threading.Event()

        # Declare and acquire `topic` parameter
        self.declare_parameter("topic", TOPIC)
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        """Start the subscriber."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(Servo, self.topic, self.subscriber_callback, qos_profile)

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        """Process a Servo message."""
        if self.msg_event_object.is_set():
            return

        self.get_logger().info("From AP : {} servo channels received".format(len(msg.channels)))

        # Verify we have channels and they're in valid range
        if len(msg.channels) > 0:
            # Check that active PWM values (non-zero) are in valid range (800-2200)
            valid = all(800 <= pwm <= 2200 for pwm in msg.channels if pwm > 0)
            if valid:
                self.get_logger().info("All active servo channels in valid PWM range (800-2200µs)")
                self.valid_channels_event.set()

        # Set event last
        self.msg_event_object.set()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_serial)
def test_dds_serial_servo_msg_recv(launch_context, launch_sitl_copter_dds_serial):
    """Test servo messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_serial
    virtual_ports = actions["virtual_ports"].action
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = ServoListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
        valid_channels_flag = node.valid_channels_event.wait(timeout=10.0)
        assert valid_channels_flag, f"Servo channels not in valid PWM range (800-2200µs)."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_servo_msg_recv(launch_context, launch_sitl_copter_dds_udp):
    """Test servo messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = ServoListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
        valid_channels_flag = node.valid_channels_event.wait(timeout=10.0)
        assert valid_channels_flag, f"Servo channels not in valid PWM range (800-2200µs)."
    finally:
        rclpy.shutdown()
    yield
