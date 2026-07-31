#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger

import numpy as np

try:
    from quanser.common import Timeout
except ImportError:
    from quanser.communications import Timeout
from qbot_localization.stream import BasicStream

class EkfLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        self.get_logger().info('Starting EKF Localization Node...')

        self.declare_parameter('init_pose', [0.0, 0.0, 0.0])
        self.init_pose = np.array(
            self.get_parameter('init_pose').get_parameter_value().double_array_value,
            dtype=np.float64)
        self.declare_parameter('scan_topic', 'scan')
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.declare_parameter('speed_topic', '/qbot_speed_feedback')
        self.speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        self.declare_parameter('imu_topic', 'imu')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.declare_parameter('pose_topic', 'pose_estimate')
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.declare_parameter('odom_topic', 'odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.declare_parameter('scan_decimation', 4)
        self.scan_decimation = self.get_parameter('scan_decimation').get_parameter_value().integer_value
        self.declare_parameter('odom_pub', True)
        self.odom_pub_flag = self.get_parameter('odom_pub').get_parameter_value().bool_value
        self.declare_parameter('tf_pub', True)
        self.tf_pub_flag = self.get_parameter('tf_pub').get_parameter_value().bool_value

        # Reset service:
        self.declare_parameter('reset_service_name', '/reset_scan_match')
        self.reset_service_name = self.get_parameter('reset_service_name').get_parameter_value().string_value
        self.declare_parameter('reset_pulse_ticks', 50)
        self.reset_pulse_ticks = int(self.get_parameter('reset_pulse_ticks').value)
        # >0 while the enable pulse is being held low; decremented each tick.
        self._reset_countdown = 0
        # True on ticks where enable is being forced low for the reset pulse.
        self._reset_tick_active = False

        # RT model input, built each timer tick: 420 ranges + 420 angles + count
        # + body_linear + body_rotation + gyro(3) + enable_bool = 847
        self.rt_input = np.zeros(847, dtype=np.float64)
        # RT model output: [x, y, yaw]
        self.rt_output_size = 3
        self.rt_stream = BasicStream(
            'tcpip://localhost:18999',
            agent='C',
            sendBufferSize=8192,
            recvBufferSize=2048,
            receiveBuffer=np.zeros(self.rt_output_size, dtype=np.float64),
            nonBlocking=True,
            reshapeOrder='C',
        )
        self.rt_recv_timeout = Timeout(seconds=0, nanoseconds=2_000_000)  # 2 ms
        self._rt_stream_init()

        self.pose = self.init_pose.copy()
        self.covariance = np.eye(3, dtype=np.float64)

        # latest sensor data, populated by subscriber callbacks
        self.latest_ranges = None        # (420,) float64
        self.latest_angles = None        # (420,) float64
        self.latest_body_linear = 0.0    # twist.linear.x
        self.latest_body_rotation = 0.0  # twist.angular.z
        self.latest_gyro = np.zeros(3, dtype=np.float64)  # angular_velocity xyz

        qos_profile = QoSProfile(depth=10)

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile)
        self.speed_sub_ = self.create_subscription(
            TwistStamped,
            self.speed_topic,
            self.speed_callback,
            20)
        self.imu_sub_ = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            20)

        self.pose_pub_ = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            qos_profile)
        if self.odom_pub_flag:
            self.odom_pub_ = self.create_publisher(
                Odometry,
                self.odom_topic,
                qos_profile)
        if self.tf_pub_flag:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.reset_srv = self.create_service(Trigger, self.reset_service_name, self.handle_reset)

        self.timer = self.create_timer(1.0 / 120, self.timer_callback) #120hz loop
                   
    def scan_callback(self, msg: LaserScan):
        step = self.scan_decimation  # 1680 -> 420
        self.latest_ranges = np.array(msg.ranges[::step], dtype=np.float64)
        self.latest_angles = np.arange(
            msg.angle_min, msg.angle_max, msg.angle_increment, dtype=np.float64)[1::step]

    def speed_callback(self, msg: TwistStamped):
        self.latest_body_linear = msg.twist.linear.x
        self.latest_body_rotation = msg.twist.angular.z

    def imu_callback(self, msg: Imu):
        self.latest_gyro[0] = msg.angular_velocity.x
        self.latest_gyro[1] = msg.angular_velocity.y
        self.latest_gyro[2] = msg.angular_velocity.z

    def handle_reset(self, request, response):
        """Trigger service callback. Schedules a one-shot low pulse on the
        RT model's `enable` input (held low for reset_pulse_ticks timer
        ticks, then released back high) and snaps the local pose estimate
        to (0, 0, 0) immediately. Runs on the same executor thread as
        timer_callback, so no lock is needed -- this just sets state that
        localize() picks up on its next tick.
        """
        self._reset_countdown = self.reset_pulse_ticks
        self.pose[0] = 0.0
        self.pose[1] = 0.0
        self.pose[2] = 0.0
        self.get_logger().info(
            f'Localization reset triggered: enable held low for {self.reset_pulse_ticks} ticks.')
        response.success = True
        response.message = 'Localization reset triggered.'
        return response

    def build_rt_input(self):
        if self.latest_ranges is None or self.latest_angles is None:
            return None
        self._reset_tick_active = self._reset_countdown > 0
        if self._reset_tick_active:
            effective_enable = 0.0
            self._reset_countdown -= 1
        else:
            effective_enable = 1.0
        self.rt_input[0:420] = self.latest_ranges
        self.rt_input[420:840] = self.latest_angles
        self.rt_input[840] = 420  # rt model expect 420 valid range points
        self.rt_input[841] = self.latest_body_linear
        self.rt_input[842] = self.latest_body_rotation
        self.rt_input[843:846] = self.latest_gyro
        self.rt_input[846] = effective_enable
        return self.rt_input

    def localize(self):
        """Synchronous send-then-receive against the RT model stream, run
        once per 120Hz timer tick. Both send() and receive() share the same
        underlying BasicStream/socket object, so they must stay on this one
        thread rather than being split across a send thread and a receive
        thread. On any failure (send error, incomplete/missing response),
        self.pose is left untouched and the caller republishes the last
        known estimate so downstream consumers still see a steady 120Hz
        stream.
        """
        rt_input = self.build_rt_input()
        if rt_input is None:
            return

        sent_flag = self.rt_stream.send(rt_input)
        if sent_flag == -1:
            self.get_logger().warn('RT model stream send failed.', throttle_duration_sec=1.0)
            return

        output = self._get_latest_data(self.rt_stream)

        # Stamp as close to receipt as this API exposes.
        self._last_receipt_time = self.get_clock().now()

        if self._reset_tick_active:
            # Enable was forced low this tick as part of a reset pulse --
            # hold the last known pose (snapped to 0,0,0 by handle_reset)
            # rather than trusting whatever the RT model returns while
            # it's mid-reset.
            return

        self.pose[0] = output[0]
        self.pose[1] = output[1]
        self.pose[2] = output[2]

    def timer_callback(self):
        self.localize()

        estimated_pose = self.pose.tolist()
        processed_x = estimated_pose[0]
        processed_y = estimated_pose[1]
        q = quaternion_from_euler(0, 0, estimated_pose[2])

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        publish_time = self.get_clock().now()
        pose_msg.header.stamp = publish_time.to_msg()
        pose_msg.pose.position.x = processed_x
        pose_msg.pose.position.y = processed_y
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub_.publish(pose_msg)

        if self.odom_pub_flag:
            odom_msg = Odometry()
            odom_msg.header.stamp = publish_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = processed_x
            odom_msg.pose.pose.position.y = processed_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            self.odom_pub_.publish(odom_msg)

        if self.tf_pub_flag:
            t = TransformStamped()
            t.header.stamp = publish_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = processed_x
            t.transform.translation.y = processed_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def _rt_stream_init(self) -> None:
        """
        Initialize the RT model stream
        """
        prev_con = False
        while not self.rt_stream.connected:
            self.rt_stream.checkConnection(timeout=self.rt_recv_timeout)
            if self.rt_stream.connected and not prev_con:
                self.get_logger().info('RT model stream connected.')
                prev_con = True
        self._flush_stream(self.rt_stream)


    def _flush_stream(self, client) -> None:
        """
        Drain any backlog of unread data waiting on the stream.

        Reads repeatedly until a read comes back empty, so the next
        real read isn't working through stale, already-superseded
        data left over from a previous command.

        Parameters
        ----------
        client : BasicStream
            The stream to flush.
        """
        flushed = 0
        while flushed == 0:
            recvFlag, bytesReceived = client.receive(
                iterations=2, timeout=self.rt_recv_timeout
            )
            if recvFlag == 0:
                flushed = 1
    
    def _get_latest_data(self, client) -> np.ndarray:
        """
        Read from the stream until the freshest available data arrives.

        Keeps reading (with a short sleep between misses) until a read
        comes back empty right after at least one successful read,
        which signals the backlog has been drained down to the most
        recent value.

        Parameters
        ----------
        client : BasicStream
            The stream to read from.

        Returns
        -------
        ndarray of shape (4,)
            The latest [speed, distance, busy, error] reading.

        Raises
        ------
        SystemExit
            If no data is received at all within `max_tries` polls.
        """
        received_data = 0
        latest_data = np.zeros((4), dtype=np.float64)

        # if client.connected:
        while True:
            recvFlag, bytesReceived = client.receive(
                iterations=2, timeout=self.rt_recv_timeout
            )
            if recvFlag:
                latest_data = client.receiveBuffer
                received_data = 1
            if recvFlag == 0:
                if received_data:
                    return latest_data
                
    def terminate(self):
        self.get_logger().info('Terminating Localization Node...')
        try:
            self.rt_stream.terminate()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    localization_node = EkfLocalizationNode()

    try:
        rclpy.spin(localization_node)
    except (ExternalShutdownException, KeyboardInterrupt):
        localization_node.terminate()
    finally:
        localization_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()