#!/usr/bin/env python3
"""
sensor_manager.py  --  GP1 Task 2.

Connects to CARLA, spawns the ego vehicle, attaches the sensor suite, and
republishes every stream on a ROS 2 topic.

This file is a SKELETON. The message-format work is already done for you in
carla_conversions.py; what is left is the part GP1 is actually about:
spawning actors, wiring every attribute to a YAML parameter, publishing a TF
tree, and shutting down cleanly.

Search for TODO. There are nine.
"""

import math

import carla
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from tf2_ros import StaticTransformBroadcaster

# The CARLA-to-ROS conversions already exist, in the package that backs the
# L2 lecture demo. They handle the buffer layouts, the left-handed to
# right-handed axis flip, and the timestamps. Import them; do not rewrite
# them. Run `ros2 run l2_carla_demo carla_bridge` to see them in use.
from l2_carla_demo.conversions import (
    image_to_msg,
    camera_info_msg,
    lidar_to_msg,
    radar_to_msg,
    gnss_to_msg,
    imu_to_msg,
    transform_from_carla,
)


class SensorManager(Node):

    def __init__(self):
        super().__init__('sensor_manager')

        # ------------------------------------------------------------------
        # Parameters. Declare everything; read nothing from a literal.
        # ------------------------------------------------------------------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout_s', 10.0)
        self.declare_parameter('synchronous_mode', True)
        self.declare_parameter('fixed_delta_seconds', 0.05)
        self.declare_parameter('town', 'Town01')
        self.declare_parameter('vehicle_blueprint', 'vehicle.tesla.model3')
        self.declare_parameter('spawn_index', 0)
        # TODO 1: declare the rest. Every attribute in carla_config.yaml that
        #         your code reads must be declared here first, including the
        #         nested camera.*, lidar.*, radar.*, gnss.* and imu.* values
        #         and the mounts.* transforms.

        self.actors = []
        self.world = None
        # The server settings as we found them. destroy() restores these, so
        # keep the original around from the moment you change anything.
        self._original_settings = None

        # ------------------------------------------------------------------
        # Connect
        # ------------------------------------------------------------------
        try:
            self.client = carla.Client(
                self.get_parameter('host').value,
                self.get_parameter('port').value)
            self.client.set_timeout(self.get_parameter('timeout_s').value)
            # Load the map explicitly. spawn_index indexes into this map's
            # spawn point list, so inheriting whatever the server had loaded
            # would silently move your vehicle somewhere else.
            self.world = self.client.load_world(
                self.get_parameter('town').value)
        except RuntimeError as exc:
            # TODO 2: a bare traceback here is the single most common reason
            #         a grader cannot run your node. Say what failed, what
            #         host and port were tried, and what to check.
            self.get_logger().error(f'could not reach CARLA: {exc}')
            raise

        self.blueprints = self.world.get_blueprint_library()

        # Everything from here on changes the server. If any of it fails,
        # hand the server back before the exception propagates; main() never
        # gets a node object to clean up when __init__ raises.
        try:
            self._setup()
        except Exception:
            self.destroy()
            raise

        self.get_logger().info('sensor_manager ready')

    def _setup(self):
        # ------------------------------------------------------------------
        # TODO 3: put the server into synchronous mode BEFORE anything else.
        #
        #   self._original_settings = self.world.get_settings()
        #   settings = self.world.get_settings()
        #   ... set synchronous_mode and fixed_delta_seconds from parameters
        #   self.world.apply_settings(settings)
        #
        # Retrofitting this later means re-checking every callback you have
        # written by then. destroy() restores _original_settings, so the
        # next person to connect does not inherit a server that will not
        # advance.
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # Publishers. One per stream, on the topics named in the GP1 table.
        # ------------------------------------------------------------------
        self.pub_rgb = self.create_publisher(
            Image, '/carla/camera/rgb/image', 10)
        self.pub_lidar = self.create_publisher(
            PointCloud2, '/carla/lidar/points', 10)
        self.pub_gnss = self.create_publisher(
            NavSatFix, '/carla/gnss/fix', 10)
        self.pub_imu = self.create_publisher(
            Imu, '/carla/imu/data', 10)
        # TODO 4: depth, semantic segmentation and radar publishers.
        #         Check the GP1 task table for the exact topic names; the
        #         grader matches on them. RADAR is a PointCloud2 carrying
        #         (x, y, z, velocity): see radar_to_msg.

        self.tf_static = StaticTransformBroadcaster(self)

        self._spawn_vehicle()
        self._publish_static_tf()
        self._attach_sensors()

        if self.get_parameter('synchronous_mode').value:
            dt = self.get_parameter('fixed_delta_seconds').value
            self.timer = self.create_timer(dt, self._tick)

    # ----------------------------------------------------------------------
    def _spawn_vehicle(self):
        """Spawn the ego vehicle at the configured spawn point."""
        # TODO 5: look up the blueprint by name from the parameter, take the
        #         spawn point at the configured index, and spawn it.
        #
        #           spawn = self.world.get_map().get_spawn_points()[idx]
        #
        #         Your assigned index comes from the GP1 team table.
        #
        #         Prefer try_spawn_actor over spawn_actor and check for None:
        #         it returns None instead of raising when the point is taken,
        #         which happens often once there is traffic.
        #
        #         Append every actor you create to self.actors.
        raise NotImplementedError('GP1 Task 2: spawn the ego vehicle.')

    # ----------------------------------------------------------------------
    def _publish_static_tf(self):
        """Publish the sensor mounts as a static TF tree.

        RViz2 cannot draw a point cloud and a camera image in the same scene
        until it knows where those sensors sit relative to each other. Skip
        this and Task 4 shows "Fixed Frame does not exist" and nothing else.
        """
        # TODO 6: build one TransformStamped per sensor from the mounts in
        #         your YAML and publish them with sendTransform().
        #
        #         The parent frame is 'ego_vehicle' (the RViz Fixed Frame).
        #         The child frame of each sensor is its key under mounts:,
        #         e.g. 'camera_rgb_front', 'lidar_top'. Use the same string
        #         as the frame_id when you publish that sensor's messages,
        #         or RViz cannot place the data.
        #
        #         Nested YAML keys become dotted parameter names, so read
        #         them with self.get_parameters_by_prefix('mounts'), which
        #         returns {'camera_rgb_front.x': Parameter, ...}.
        #
        #         transform_from_carla(carla_transform, parent, child,
        #                              sim_time) does the conversion for you;
        #         sim_time is world.get_snapshot().timestamp.elapsed_seconds.
        #
        #         Check the result with: ros2 run tf2_tools view_frames
        raise NotImplementedError('GP1 Task 2: publish the static TF tree.')

    # ----------------------------------------------------------------------
    def _attach_sensors(self):
        """Attach every sensor in the suite to the ego vehicle."""
        self._attach_rgb_camera()
        # TODO 7: the other six. Each one follows the same four steps as the
        #         camera below: find the blueprint, set its attributes from
        #         parameters, build the carla.Transform from the matching
        #         entry in mounts, and spawn with attach_to=self.vehicle.
        #
        #         Use carla.AttachmentType.Rigid. SpringArm smooths the
        #         sensor's motion for video, which makes the sensor-to-vehicle
        #         transform non-constant and invalidates your Task 5
        #         extrinsic.
        #
        #         The semantic camera's raw image holds the class tag in the
        #         red channel and looks black in RViz. Call
        #         measurement.convert(carla.ColorConverter.CityScapesPalette)
        #         before image_to_msg to publish it in colour.

    def _attach_rgb_camera(self):
        """Worked example. The remaining sensors follow this shape."""
        bp = self.blueprints.find('sensor.camera.rgb')
        # TODO 8: set image_size_x, image_size_y, fov and sensor_tick from
        #         the camera.* parameters rather than from literals.

        mount = carla.Transform(
            carla.Location(x=1.5, z=1.6),     # TODO: from mounts.camera_rgb_front
            carla.Rotation())

        sensor = self.world.spawn_actor(
            bp, mount, attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid)
        sensor.listen(self._rgb_callback)
        self.actors.append(sensor)

    def _rgb_callback(self, measurement):
        """Republish one camera frame.

        Note what image_to_msg does NOT do: it never touches the wall clock.
        The timestamp comes off the measurement, because that records when
        the data was captured rather than when Python got round to the
        callback. Task 3's timing analysis is meaningless otherwise.
        """
        self.pub_rgb.publish(
            image_to_msg(measurement, 'camera_rgb_front', Image))

    # ----------------------------------------------------------------------
    def _tick(self):
        """Advance the simulation by one step. Synchronous mode only."""
        self.world.tick()

    # ----------------------------------------------------------------------
    def destroy(self):
        """Destroy every actor and hand the server back as we found it."""
        self.get_logger().info(f'destroying {len(self.actors)} actors')
        # TODO 9: stop the sensors listening (sensor.stop()) before you
        #         destroy them, and destroy every actor in reverse order.
        #         Leaked sensors keep consuming server resources until the
        #         map is reloaded.
        for actor in reversed(self.actors):
            if actor is not None and actor.is_alive:
                actor.destroy()
        self.actors.clear()

        # Hand the clock back. Left in synchronous mode with nothing ticking
        # it, the server looks frozen to the next client.
        if self.world is not None and self._original_settings is not None:
            self.world.apply_settings(self._original_settings)
            self._original_settings = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
