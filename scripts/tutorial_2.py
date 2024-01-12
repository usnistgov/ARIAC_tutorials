#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_2.py
'''

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface(enable_moveit=False)
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    interface.start_competition()

    count = 0

    while rclpy.ok():
        try:
            if interface.conveyor_part_count > count:
                interface.get_logger().info(f"Part detected on conveyor. Count: {interface.conveyor_part_count}")
                count+= 1
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()
