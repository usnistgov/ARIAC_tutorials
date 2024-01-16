#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_3.py
'''
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

    while rclpy.ok():
        try:
            # publishes parts in the bin on /ariac/sensors/slot_occupancy
            interface.get_bin_parts(bin_number)
            sleep(1)
        except KeyboardInterrupt:
            break
            
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()