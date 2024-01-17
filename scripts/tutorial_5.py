#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_msgs.msg import Order as OrderMsg
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface(enable_moveit=False)
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    interface.start_competition()

    # Wait for assembly order to be recieved
    while True:
        interface.get_logger().info("Waiting for assembly order...", once=True)
        if OrderMsg.ASSEMBLY in [order.order_type for order in interface.orders]:
            interface.get_logger().info("Assembly order recieved...", once=True)
            break

    assembly_order = interface.orders[-1]
    for agv in assembly_order.order_task.agv_numbers:
        interface.lock_agv_tray(agv)
        interface.move_agv_to_station(agv, assembly_order.order_task.station)

if __name__ == '__main__':
    main()