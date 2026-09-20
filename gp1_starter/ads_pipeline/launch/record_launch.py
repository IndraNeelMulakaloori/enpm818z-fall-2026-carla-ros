"""Task 3: start the sensor suite and record every topic to a rosbag.

TODO. Two ways to do it, both acceptable:

  1. Include sensors_launch.py, then add an ExecuteProcess running
     `ros2 bag record` with the topic list.
  2. Use rosbag2_py from inside a node of your own.

Whichever you choose, the bag must contain all seven sensor topics and run
for at least two minutes. Check with `ros2 bag info` before you submit; a
bag missing topics is the most common way to lose marks on this task.
"""

from launch import LaunchDescription


def generate_launch_description():
    raise NotImplementedError('GP1 Task 3: write the recording launch file.')
    return LaunchDescription([])
