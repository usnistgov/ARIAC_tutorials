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
    
    looking_for_parts_in_bin = 2
    while(1):
        # publishes parts in the bin on /ariac/sensors/slot_occupancy
        interface.get_bin_parts(looking_for_parts_in_bin)
        sleep(1)
            
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()