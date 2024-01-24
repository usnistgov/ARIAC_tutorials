#!/usr/bin/env python3
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_tutorials.competition_interface import CompetitionInterface
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface(enable_moveit=False)
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    interface.start_competition()

    # Turns on a debug topic to visualize bounding boxes and slots
    # /ariac/sensors/display_bounding_boxes
    interface.display_bounding_boxes = True
    
    bin_number = 2

    interface.get_logger().info(f"Getting parts from bin {bin_number}")
    bin_parts = None
    while bin_parts is None:
        bin_parts = interface.get_bin_parts(bin_number)
        sleep(1)
    if bin_parts:
        for _slot_number, _part in bin_parts.items():
            if _part.type is None:
                interface.get_logger().info(f"Slot {_slot_number}: Empty")
            else:
                interface.get_logger().info(f"Slot {_slot_number}: {_part.color} {_part.type}")
    
    interface.end_competition()
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()