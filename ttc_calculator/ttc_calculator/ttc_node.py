import math
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

class TTCCalculatorNode(Node):
    def __init__(self):
        super().__init__('ttc_calculator_node_py')

        # Parameters
        self.declare_parameter('pointcloud_topic', '/ouster/points')
        self.declare_parameter('twist_topic', '/vehicle/twist')
        self.declare_parameter('fov_angle_deg', 10.0)              # total
        self.declare_parameter('ground_z_threshold', 0.2)          # meters
        self.declare_parameter('min_velocity_threshold', 0.1)      # m/s
        self.declare_parameter('min_cluster_points', 5)            # pts near dmin
        self.declare_parameter('cluster_radius', 0.5)              # m window around dmin
        self.declare_parameter('csv_filename', 'ttc_data.csv')
        self.declare_parameter('marker_use_rotated', True)
        self.declare_parameter('yaw_offset_deg', 0.0)

        cloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self.half_fov = math.radians(self.get_parameter('fov_angle_deg').get_parameter_value().double_value) / 2.0
        self.ground_z = self.get_parameter('ground_z_threshold').get_parameter_value().double_value
        self.min_vel = self.get_parameter('min_velocity_threshold').get_parameter_value().double_value
        self.min_cluster = int(self.get_parameter('min_cluster_points').get_parameter_value().integer_value)
        self.cluster_radius = self.get_parameter('cluster_radius').get_parameter_value().double_value
        self.csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        self.marker_use_rotated = bool(self.get_parameter('marker_use_rotated').get_parameter_value().bool_value)
        self.yaw_offset = math.radians(self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)
        self.cos_yaw = math.cos(self.yaw_offset)
        self.sin_yaw = math.sin(self.yaw_offset)

        # Output CSV
        try:
            self.csv = open(self.csv_filename, 'w', newline='')
            self.writer = csv.writer(self.csv)
            self.writer.writerow(['timestamp', 'velocity', 'distance', 'ttc'])
        except Exception as e:
            self.get_logger().error(f'Failed to open CSV: {e}')
            self.csv = None

        # Runtime state for prints/markers
        self.last_velocity = 0.0
        self.last_distance = float('nan')
        self.last_ttc = float('nan')
        self.last_point = None           # (x,y,z) of nearest obstacle
        self.last_frame = ''

        # Subscriptions
        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST

        self.sub_twist = self.create_subscription(TwistStamped, twist_topic, self.on_twist, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_sensor)

        # Live numeric publishers
        self.pub_vel = self.create_publisher(Float64, 'ttc/velocity', 10)
        self.pub_dist = self.create_publisher(Float64, 'ttc/distance', 10)
        self.pub_ttc  = self.create_publisher(Float64, 'ttc/ttc', 10)

        # RViz marker publisher
        self.pub_marker = self.create_publisher(Marker, 'ttc/marker', 10)

        # 2 Hz terminal report
        self.report_timer = self.create_timer(0.5, self._report_status)

        self.get_logger().info(f'TTC node started (py). Cloud: {cloud_topic}, Twist: {twist_topic}')


    def _rotate_xy(self, x, y):
        # Rotate by yaw_offset: positive yaw rotates the frame counterclockwise
        xr = x * self.cos_yaw + y * self.sin_yaw
        yr = -x * self.sin_yaw + y * self.cos_yaw
        return xr, yr


    # -- Callbacks --
    def on_twist(self, msg: TwistStamped):
        self.last_velocity = float(msg.twist.linear.x)

    def on_cloud(self, msg: PointCloud2):
        # First pass: find minimum distance and its point
        dmin = math.inf
        pmin = None
        for x, y, z, *_ in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            x, y, z = float(x), float(y), float(z)
            xr, yr = self._rotate_xy(x, y)
            if xr <= 0.0:
                continue
            ang = math.atan2(yr, xr)
            if abs(ang) > self.half_fov:
                continue
            if z < self.ground_z:
                continue
            d = math.sqrt(x*x + y*y + z*z)
            if d < dmin:
                dmin = d
                pmin = (x, y, z)

        # Second pass: count neighbors near dmin for noise rejection
        near = 0
        if math.isfinite(dmin):
            for x, y, z, *_ in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                x, y, z = float(x), float(y), float(z)
                xr, yr = self._rotate_xy(x, y)
                if xr <= 0.0:
                    continue
                ang = math.atan2(yr, xr)
                if abs(ang) > self.half_fov:
                    continue
                if z < self.ground_z:
                    continue
                d = math.sqrt(x*x + y*y + z*z)
                if abs(d - dmin) <= self.cluster_radius:
                    near += 1

        if (not math.isfinite(dmin)) or (near < self.min_cluster):
            dmin = float('inf')
            pmin = None

        v = self.last_velocity
        ttc = float('inf')
        if math.isfinite(dmin) and v > self.min_vel:
            ttc = dmin / v

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.csv:
            self.writer.writerow([f'{ts:.3f}', f'{v:.3f}', 'inf' if not math.isfinite(dmin) else f'{dmin:.3f}',
                                  'inf' if not math.isfinite(ttc) else f'{ttc:.3f}'])

        # Publish numeric topics
        fv = Float64(); fv.data = float(v)
        fd = Float64(); fd.data = float('nan') if not math.isfinite(dmin) else float(dmin)
        ft = Float64(); ft.data = float('nan') if not math.isfinite(ttc) else float(ttc)
        self.pub_vel.publish(fv)
        self.pub_dist.publish(fd)
        self.pub_ttc.publish(ft)

        # Publish RViz markers
        self.last_point = pmin
        self.last_frame = msg.header.frame_id
        self._publish_markers(v, dmin, ttc, pmin, msg.header.frame_id)

        # Save for report
        self.last_distance = dmin
        self.last_ttc = ttc

    # -- Helpers --
    def _publish_markers(self, v, d, ttc, pmin, frame):
        # Text marker with v/d/ttc
        text = f'v={v:.2f} m/s\\n' + (f'd={d:.2f} m\\n' if math.isfinite(d) else 'd=inf\\n') + \
               (f'ttc={ttc:.2f} s' if math.isfinite(ttc) else 'ttc=inf')
        m_text = Marker()
        m_text.header.frame_id = frame
        m_text.header.stamp = self.get_clock().now().to_msg()
        m_text.ns = 'ttc'
        m_text.id = 2
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = 0.0
        m_text.pose.position.y = 0.0
        m_text.pose.position.z = 1.5
        m_text.scale.z = 0.35   # font size
        m_text.color.a = 1.0
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.text = text
        self.pub_marker.publish(m_text)

        # Line marker from origin to pmin
        m_line = Marker()
        m_line.header.frame_id = frame
        m_line.header.stamp = m_text.header.stamp
        m_line.ns = 'ttc'
        m_line.id = 1
        if pmin is None:
            m_line.action = Marker.DELETE
        else:
            m_line.action = Marker.ADD
            m_line.type = Marker.LINE_STRIP
            m_line.scale.x = 0.05  # line width
            m_line.color.a = 1.0
            m_line.color.r = 0.0
            m_line.color.g = 1.0
            m_line.color.b = 0.0
            p0 = Point(); p0.x = 0.0; p0.y = 0.0; p0.z = 0.0
            p1 = Point()
            if self.marker_use_rotated:
                rx, ry = self._rotate_xy(pmin[0], pmin[1])
                p1.x, p1.y, p1.z = rx, ry, pmin[2]
            else:
                p1.x, p1.y, p1.z = pmin
            m_line.points = [p0, p1]
        self.pub_marker.publish(m_line)

    def _report_status(self):
        def fmt(x):
            if isinstance(x, float):
                if math.isinf(x): return 'inf'
                if math.isnan(x): return 'nan'
                return f'{x:.2f}'
            return str(x)

        self.get_logger().info(f"[TTC] v={fmt(self.last_velocity)} m/s, d={fmt(self.last_distance)} m, ttc={fmt(self.last_ttc)} s")

def main():
    rclpy.init()
    node = TTCCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, 'csv') and node.csv:
                node.csv.flush()
                node.csv.close()
                node.get_logger().info(f'Wrote CSV to {os.path.abspath(node.csv_filename)}')
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
