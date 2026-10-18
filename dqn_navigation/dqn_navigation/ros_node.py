"""ROS 2 interface: /goal_pose + /scan + TF -> learned /cmd_vel actions."""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .agent import DQNAgent


class DQNController(Node):
    # Keep turns slow enough for the 5 Hz simulated LiDAR to register clean,
    # overlapping scans while the exploratory policy changes actions.
    ACTIONS = ((0.10, 0.0), (0.0, 0.35), (0.0, -0.35),
               (0.08, 0.22), (0.08, -0.22), (-0.04, 0.0))
    LASER_BINS = 24
    STATE_SIZE = LASER_BINS + 4

    def __init__(self):
        super().__init__('dqn_controller')
        self.declare_parameter('learning', True)
        self.declare_parameter('checkpoint', os.path.expanduser('~/.ros/turtlebot3_dqn.pt'))
        self.declare_parameter('control_rate', 5.0)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('collision_distance', 0.13)
        self.declare_parameter('safety_distance', 0.30)
        self.learning = bool(self.get_parameter('learning').value)
        checkpoint = os.path.expanduser(str(self.get_parameter('checkpoint').value))
        os.makedirs(os.path.dirname(checkpoint), exist_ok=True)
        self.agent = DQNAgent(self.STATE_SIZE, len(self.ACTIONS), checkpoint, self.learning)
        self.checkpoint = checkpoint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal = None
        self.scan = None
        self.last_action = 0
        self.previous_state = None
        self.previous_distance = None
        self.previous_action = None
        self.episode_steps = 0
        self.episodes = 0
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(MarkerArray, '/dqn/status_marker', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(1.0 / float(self.get_parameter('control_rate').value), self.control_step)
        mode = 'training' if self.learning else 'inference'
        self.get_logger().info(f'DQN {mode} controller ready; checkpoint: {checkpoint}')

    def goal_callback(self, msg):
        self.goal = msg
        self.previous_state = self.previous_distance = self.previous_action = None
        self.episode_steps = 0
        self.get_logger().info('Received clicked goal; starting a new DQN episode.')

    def clicked_point_callback(self, msg):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position = msg.point
        goal.pose.orientation.w = 1.0
        self.goal_callback(goal)

    def scan_callback(self, msg):
        self.scan = msg

    def make_state(self):
        if self.goal is None or self.scan is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal.header.frame_id or 'map', 'base_footprint', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
        except TransformException:
            return None
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        gx = self.goal.pose.position.x - x
        gy = self.goal.pose.position.y - y
        distance = math.hypot(gx, gy)
        bearing = math.atan2(math.sin(math.atan2(gy, gx) - yaw),
                             math.cos(math.atan2(gy, gx) - yaw))
        ranges = np.asarray(self.scan.ranges, dtype=np.float32)
        max_range = self.scan.range_max if math.isfinite(self.scan.range_max) else 3.5
        ranges[~np.isfinite(ranges)] = max_range
        ranges = np.clip(ranges, self.scan.range_min, max_range)
        angles = self.scan.angle_min + np.arange(len(ranges), dtype=np.float32) * self.scan.angle_increment
        front = ranges[np.abs(angles) <= 0.45]
        left = ranges[(angles > 0.45) & (angles <= 1.25)]
        right = ranges[(angles < -0.45) & (angles >= -1.25)]
        front_min = float(np.min(front)) if len(front) else max_range
        left_clearance = float(np.min(left)) if len(left) else max_range
        right_clearance = float(np.min(right)) if len(right) else max_range
        sectors = np.array([np.min(s) if len(s) else max_range
                            for s in np.array_split(ranges, self.LASER_BINS)], dtype=np.float32)
        sectors /= max(max_range, 1e-3)
        state = np.concatenate((sectors, np.array([
            min(distance / 5.0, 1.0), math.sin(bearing), math.cos(bearing),
            self.last_action / max(1, len(self.ACTIONS) - 1)], dtype=np.float32)))
        return state, distance, float(np.min(ranges)), front_min, left_clearance, right_clearance

    def publish_training_marker(self, distance):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_footprint'
        marker.ns = 'dqn_training'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.48
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.85, 0.1, 1.0
        marker.lifetime.sec = 1
        marker.text = (f'DQN TRAIN | episode {self.episodes + 1} | step {self.episode_steps} '
                       f'| updates {self.agent.steps} | epsilon {self.agent.epsilon:.2f} '
                       f'| goal {distance:.2f} m')
        self.status_pub.publish(MarkerArray(markers=[marker]))

    def control_step(self):
        if self.goal is None:
            return
        result = self.make_state()
        if result is None:
            return
        state, distance, nearest, front_min, left_clearance, right_clearance = result
        if distance <= float(self.get_parameter('goal_tolerance').value):
            self.cmd_pub.publish(Twist())
            if self.previous_state is not None:
                self.agent.remember(self.previous_state, self.previous_action, 50.0, state, True)
                self.agent.train_step()
            self.agent.save()
            self.episodes += 1
            self.get_logger().info(f'Goal reached; completed episode {self.episodes}.')
            self.goal = None
            return
        reward = -0.05
        done = False
        if self.previous_state is not None:
            reward += 2.0 * (self.previous_distance - distance)
            collision_distance = float(self.get_parameter('collision_distance').value)
            moved_toward_obstacle = (self.previous_action is not None and
                                     self.ACTIONS[self.previous_action][0] > 0.0)
            if front_min < collision_distance and moved_toward_obstacle:
                reward -= 20.0
                done = True
            self.agent.remember(self.previous_state, self.previous_action, reward, state, done)
            self.agent.train_step()
            if done:
                self.cmd_pub.publish(Twist())
                self.agent.save()
                self.get_logger().warn(
                    f'Front near-contact termination at {front_min:.3f} m; send a new clicked goal to continue training.')
                self.goal = None
                self.previous_state = self.previous_distance = self.previous_action = None
                return
        action = self.agent.act(state)
        safety_distance = float(self.get_parameter('safety_distance').value)
        linear, angular = self.ACTIONS[action]
        unsafe_forward = linear > 0.0 and (
            front_min < safety_distance
            or (angular > 0.0 and left_clearance < safety_distance)
            or (angular < 0.0 and right_clearance < safety_distance))
        if unsafe_forward:
            action = 1 if left_clearance >= right_clearance else 2
        linear, angular = self.ACTIONS[action]
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)
        self.previous_state = state
        self.previous_distance = distance
        self.previous_action = action
        self.last_action = action
        self.episode_steps += 1
        if self.episode_steps % 5 == 0:
            self.publish_training_marker(distance)
        if self.episode_steps % 25 == 0:
            self.get_logger().info(
                f'Training episode {self.episodes + 1}: step {self.episode_steps}, '
                f'optimizer updates {self.agent.steps}, epsilon {self.agent.epsilon:.3f}, '
                f'goal distance {distance:.2f} m.')
        if self.episode_steps >= 1500:
            self.cmd_pub.publish(Twist())
            self.agent.save()
            self.get_logger().warn('Episode ended; send a new clicked goal to continue training.')
            self.goal = None
            self.previous_state = self.previous_distance = self.previous_action = None


def main(args=None):
    rclpy.init(args=args)
    node = DQNController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.agent.save()
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
