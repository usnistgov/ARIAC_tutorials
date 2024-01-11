#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_5.py
'''


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

    while not interface.orders:
        try:
            rclpy.spin_once(interface)
        except KeyboardInterrupt:
            break

    for order in interface.orders:
        if order.order_type == OrderMsg.ASSEMBLY:
            for agv in order.order_task.agv_numbers:
                interface.lock_agv_tray(agv)
                interface.move_agv_to_station(agv, order.order_task.station)

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()