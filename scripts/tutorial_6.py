#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials dev_mode:=True
- ros2 launch ariac_tutorials tutorial_6.launch.py
'''

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_msgs.msg import Order as OrderMsg
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
            for part in order.order_task.parts:
                interface.get_logger().info(f"Picking up {interface._part_colors[part.part.color]} {interface._part_types[part.part.type]}")
                interface.floor_robot_pick_bin_part(part.part)
                break
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
