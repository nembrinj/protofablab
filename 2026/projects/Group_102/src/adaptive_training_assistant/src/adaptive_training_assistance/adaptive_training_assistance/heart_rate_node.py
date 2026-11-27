#!/usr/bin/env python3
"""Real heart-rate sensor: reads a Polar chest strap over Bluetooth (BLE).

Finds the sensor by name, subscribes to the standard Bluetooth "Heart Rate
Measurement" characteristic and publishes every value on /heart_rate, where
training_logic and the Node-RED dashboard read it. If the sensor is not
found or disconnects, it keeps searching.

Output: /heart_rate (Float32, bpm)
Needs the `bleak` Python package; on macOS the terminal needs Bluetooth access.
"""

import asyncio
import threading

import rclpy
from bleak import BleakClient, BleakScanner
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

DEVICE_NAME = "Polar"  # Part of the sensor's Bluetooth name.
# Standard Bluetooth "Heart Rate Measurement" characteristic.
HEART_RATE_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
SCAN_SECONDS = 10.0
RETRY_SECONDS = 3.0


def parse_heart_rate(data):
    """bpm from a Heart Rate Measurement packet: if bit 0 of the first byte
    (flags) is set the value has 16 bits, otherwise 8 bits."""
    if data[0] & 0x01:
        return int.from_bytes(data[1:3], byteorder="little")
    return data[1]


class HeartRateNode(Node):
    def __init__(self):
        super().__init__("heart_rate")
        self.heart_rate_pub = self.create_publisher(Float32, "/heart_rate", 10)
        # Bluetooth runs in its own thread (asyncio), next to the ROS spin.
        threading.Thread(target=lambda: asyncio.run(self.ble_loop()), daemon=True).start()

    def on_measurement(self, sender, data):
        heart_rate = parse_heart_rate(data)
        self.heart_rate_pub.publish(Float32(data=float(heart_rate)))
        self.get_logger().debug(f"Heart rate: {heart_rate} bpm")

    async def ble_loop(self):
        while True:
            try:
                self.get_logger().info(f"Searching for a '{DEVICE_NAME}' sensor...")
                devices = await BleakScanner.discover(timeout=SCAN_SECONDS)
                sensor = next(
                    (d for d in devices if d.name and DEVICE_NAME.lower() in d.name.lower()),
                    None,
                )
                if sensor is None:
                    self.get_logger().warning("Sensor not found, searching again.")
                    await asyncio.sleep(RETRY_SECONDS)
                    continue

                async with BleakClient(sensor) as client:
                    self.get_logger().info(f"Connected to {sensor.name}")
                    await client.start_notify(HEART_RATE_UUID, self.on_measurement)
                    while client.is_connected:
                        await asyncio.sleep(1.0)
                self.get_logger().warning("Sensor disconnected, searching again.")
            except Exception as error:  # Bluetooth errors: retry, never crash.
                self.get_logger().warning(f"Bluetooth error: {error}")
                await asyncio.sleep(RETRY_SECONDS)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeartRateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
