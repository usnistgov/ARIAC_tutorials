#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_msgs.msg import Order as OrderMsg, KittingTask as KT
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    interface.start_competition()
    
    interface.add_objects_to_planning_scene()
    
    for order in interface.orders:
        if order.order_type == OrderMsg.KITTING:
            interface.floor_robot_pick_and_place_tray(order.order_task.tray_id, order.order_task.agv_number)
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
