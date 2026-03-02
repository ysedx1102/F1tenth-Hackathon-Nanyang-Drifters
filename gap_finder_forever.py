#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
 
        # ============ PARAMETERS ============
        self.declare_parameter('base_lookahead', 3.5)
        self.declare_parameter('robot_width', 0.2032)
        self.declare_parameter('base_obstacle_bubble', 0.35)
        self.declare_parameter('disparity_threshold', 0.3)
        self.declare_parameter('max_speed_fast', 7.5)      # Insane speed for first laps
        self.declare_parameter('max_speed_stable', 5.2)    # Safe speed after
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_steering', 0.34)
        self.declare_parameter('disparity_bubble_radius', 0.2)
        self.declare_parameter('steering_gain_fast', 1.0)  # Aggressive steering
        self.declare_parameter('steering_gain_stable', 0.78)  # Stable steering
        self.declare_parameter('field_of_vision', np.pi * 0.6)
        self.declare_parameter('lidarscan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        
        # Performance parameters
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('smooth_steering', True)
        self.declare_parameter('steering_smoothing_alpha', 0.3)
        self.declare_parameter('gap_width_bonus', 0.5)
        
        # Corner-specific parameters
        self.declare_parameter('corner_slowdown_angle', 0.15)
        self.declare_parameter('corner_speed_factor_fast', 0.85)  # Fast corner speed
        self.declare_parameter('corner_speed_factor_stable', 0.6)  # Stable corner speed
        self.declare_parameter('exit_speed_boost_fast', 1.8)      # Fast exit
        self.declare_parameter('exit_speed_boost_stable', 1.35)   # Stable exit
        self.declare_parameter('min_turn_radius', 0.8)

        # ============ LAP COUNT PARAMETERS ============
        self.declare_parameter('fast_laps', 2)  # Number of all-out fast laps
        self.declare_parameter('switch_mode_after', 2)  # Switch after lap X

        # Get all parameters
        self.base_lookahead = self.get_parameter('base_lookahead').value
        self.robot_width = self.get_parameter('robot_width').value
        self.base_obstacle_bubble = self.get_parameter('base_obstacle_bubble').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.max_speed_fast = self.get_parameter('max_speed_fast').value
        self.max_speed_stable = self.get_parameter('max_speed_stable').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering = self.get_parameter('max_steering').value
        self.disparity_bubble_radius = self.get_parameter('disparity_bubble_radius').value
        self.steering_gain_fast = self.get_parameter('steering_gain_fast').value
        self.steering_gain_stable = self.get_parameter('steering_gain_stable').value
        self.field_of_vision = self.get_parameter('field_of_vision').value
        self.lidar_scan_topic = self.get_parameter('lidarscan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.smooth_steering = self.get_parameter('smooth_steering').value
        self.steering_alpha = self.get_parameter('steering_smoothing_alpha').value
        self.gap_width_bonus = self.get_parameter('gap_width_bonus').value
        self.corner_slowdown_angle = self.get_parameter('corner_slowdown_angle').value
        self.corner_speed_factor_fast = self.get_parameter('corner_speed_factor_fast').value
        self.corner_speed_factor_stable = self.get_parameter('corner_speed_factor_stable').value
        self.exit_speed_boost_fast = self.get_parameter('exit_speed_boost_fast').value
        self.exit_speed_boost_stable = self.get_parameter('exit_speed_boost_stable').value
        self.min_turn_radius = self.get_parameter('min_turn_radius').value
        self.fast_laps = self.get_parameter('fast_laps').value
        self.switch_mode_after = self.get_parameter('switch_mode_after').value

        # State variables
        self.prev_steering = 0.0
        self.current_speed = 1.0
        self.scan_count = 0
        self.angle_increment = None
        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap_time = float('inf')
        self.fastest_lap_achieved = False
        
        # Corner state tracking
        self.corner_state = 'straight'
        self.corner_direction = 0
        self.consecutive_turn_frames = 0
        
        # History for smoothing
        self.steering_history = deque(maxlen=10)
        
        # Publishers/Subscribers
        self.subscriber = self.create_subscription(LaserScan, 
                                                   self.lidar_scan_topic,
                                                   self.lidar_callback,
                                                   10)
        
        self.publisher = self.create_publisher(AckermannDriveStamped,
                                                self.drive_topic,
                                                10)

        # Visualization
        self.gap_viz_publisher = self.create_publisher(Marker, "/goal_point", 10)
        
        self.get_logger().info("="*70)
        self.get_logger().info("🏁 GAP FOLLOWER - FAST FIRST, STABLE FOREVER 🏁")
        self.get_logger().info("="*70)
        self.get_logger().info(f"Fast Mode (Laps 1-{self.fast_laps}): {self.max_speed_fast} m/s")
        self.get_logger().info(f"Stable Mode (Lap {self.switch_mode_after}+): {self.max_speed_stable} m/s")
        self.get_logger().info("="*70)

    def get_current_mode(self):
        """Return 'fast' or 'stable' based on lap count"""
        if self.lap_count < self.fast_laps:
            return 'fast'
        else:
            return 'stable'

    def preprocess_lidar(self, data):
        """Preprocess LiDAR"""
        self.angle_increment = data.angle_increment
        
        ranges = np.array(data.ranges)
        ranges = np.clip(ranges, 0.1, 30.0)
        
        fov_indices = int(self.field_of_vision / data.angle_increment)
        center_index = int(abs(data.angle_min) / data.angle_increment)
        fov_start_i = max(0, center_index - fov_indices)
        fov_end_i = min(len(ranges)-1, center_index + fov_indices)

        limited_ranges = np.copy(ranges)
        limited_ranges[:fov_start_i] = 0
        limited_ranges[fov_end_i+1:] = 0

        valid_ranges = limited_ranges[limited_ranges > 0]
        if len(valid_ranges) == 0:
            return limited_ranges, fov_start_i, fov_end_i, center_index
            
        nearest_obstacle_distance = np.min(valid_ranges)
        nearest_idx = np.where(limited_ranges == nearest_obstacle_distance)[0][0]

        if nearest_obstacle_distance < 2.0:
            obs_bubble_half_angle = np.arctan(self.base_obstacle_bubble / nearest_obstacle_distance)
            bubble_extension = int(np.ceil(obs_bubble_half_angle / data.angle_increment))
            
            start = max(0, nearest_idx - bubble_extension)
            end = min(len(limited_ranges)-1, nearest_idx + bubble_extension)
            limited_ranges[start:end] = 0

        proc_ranges = np.copy(limited_ranges)
        proc_ranges = np.where(proc_ranges > self.base_lookahead, self.base_lookahead, proc_ranges)

        return proc_ranges, fov_start_i, fov_end_i, center_index

    def detect_corner(self, proc_ranges, center_idx):
        """Detect corners"""
        left_idx = max(0, center_idx - 40)
        right_idx = min(len(proc_ranges)-1, center_idx + 40)
        
        left_dist = proc_ranges[left_idx] if proc_ranges[left_idx] > 0 else 0
        right_dist = proc_ranges[right_idx] if proc_ranges[right_idx] > 0 else 0
        center_dist = proc_ranges[center_idx]
        
        if left_dist < center_dist * 0.6:
            return 'right_turn', 1
        elif right_dist < center_dist * 0.6:
            return 'left_turn', -1
        else:
            return 'straight', 0

    def lap_timing(self):
        """Simple lap timing"""
        current_time = self.get_clock().now()
        
        if self.lap_start_time is None:
            self.lap_start_time = current_time
            return
        
        lap_duration = (current_time - self.lap_start_time).nanoseconds / 1e9
        threshold = 120.0 if self.lap_count == 0 else 45.0
        
        if lap_duration > threshold:
            self.lap_count += 1
            
            if lap_duration < self.best_lap_time and self.lap_count > 0:
                self.best_lap_time = lap_duration
                self.fastest_lap_achieved = True
                self.get_logger().info(f"🔥 FASTEST LAP: {lap_duration:.2f}s (Lap {self.lap_count})")
            else:
                self.get_logger().info(f"Lap {self.lap_count}: {lap_duration:.2f}s")
            
            # Log mode switch
            if self.lap_count == self.switch_mode_after:
                self.get_logger().info("="*70)
                self.get_logger().info(f"🛑 SWITCHING TO STABLE MODE (Target: 1:30)")
                self.get_logger().info("="*70)
            
            self.lap_start_time = current_time

    def find_best_gap(self, proc_ranges, fov_start_i, fov_end_i, center_idx, corner_info):
        """Find best gap"""
        best_score = -1
        best_idx = center_idx
        corner_type, direction = corner_info
        
        for i in range(fov_start_i, fov_end_i):
            if proc_ranges[i] > 0:
                dist_score = proc_ranges[i] / self.base_lookahead
                angle_from_center = abs(i - center_idx) * self.angle_increment
                angle_score = np.cos(angle_from_center) ** 2
                
                corner_bias = 1.0
                if corner_type != 'straight':
                    target_idx = center_idx + direction * 20
                    distance_to_target = abs(i - target_idx) / 100
                    corner_bias = 1.0 - min(distance_to_target, 0.5)
                
                score = dist_score * 0.5 + angle_score * 0.3 + corner_bias * 0.2
                
                if score > best_score:
                    best_score = score
                    best_idx = i
        
        return best_idx

    def calculate_speed(self, steering_angle, corner_info, proc_ranges, center_idx):
        """Calculate speed based on current mode"""
        corner_type, _ = corner_info
        steer_abs = abs(steering_angle)
        
        # Get mode-specific parameters
        mode = self.get_current_mode()
        
        if mode == 'fast':
            max_speed = self.max_speed_fast
            corner_factor = self.corner_speed_factor_fast
            exit_boost = self.exit_speed_boost_fast
        else:
            max_speed = self.max_speed_stable
            corner_factor = self.corner_speed_factor_stable
            exit_boost = self.exit_speed_boost_stable
        
        # Simple speed calculation
        if corner_type == 'straight':
            if proc_ranges[center_idx] > self.base_lookahead * 0.8:
                target_speed = max_speed
            else:
                target_speed = max_speed * 0.8
        else:
            # In corners, slow down based on steering
            corner_speed = max_speed * corner_factor
            tightness = 1.0 - min(steer_abs / self.max_steering, 0.5)
            target_speed = max(self.min_speed, corner_speed * tightness)
        
        # Smooth speed changes
        speed_diff = target_speed - self.current_speed
        self.current_speed += np.clip(speed_diff * 0.1, -0.3, 0.3)
        
        return self.current_speed

    def lidar_callback(self, data):
        """Main callback"""
        self.scan_count += 1
        
        # Lap timing
        if self.scan_count % 100 == 0:
            self.lap_timing()
        
        # Preprocess
        proc_ranges, fov_start_i, fov_end_i, center_idx = self.preprocess_lidar(data)

        # Detect corner
        corner_info = self.detect_corner(proc_ranges, center_idx)
        corner_type, direction = corner_info

        # Find best gap
        best_idx = self.find_best_gap(proc_ranges, fov_start_i, fov_end_i, center_idx, corner_info)

        # Calculate steering
        target_angle = data.angle_min + best_idx * data.angle_increment
        
        # Use mode-specific steering gain
        mode = self.get_current_mode()
        steering_gain = self.steering_gain_fast if mode == 'fast' else self.steering_gain_stable
        
        steering_angle = target_angle * steering_gain

        # Smooth steering
        if self.smooth_steering:
            steering_angle = (self.steering_alpha * steering_angle +
                              (1 - self.steering_alpha) * self.prev_steering)
        
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        self.prev_steering = steering_angle

        # Calculate speed
        target_speed = self.calculate_speed(steering_angle, corner_info, proc_ranges, center_idx)

        # Publish
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(target_speed)
        self.publisher.publish(msg)

        # Visualization
        if self.scan_count % 10 == 0:
            goal_coord = np.array([
                proc_ranges[best_idx] * np.cos(data.angle_min + best_idx * data.angle_increment),
                proc_ranges[best_idx] * np.sin(data.angle_min + best_idx * data.angle_increment)
            ])
            self.visualize_goal(goal_coord)

        # Debug
        if self.debug_mode and self.scan_count % 100 == 0:
            mode_display = "🔥 FAST" if mode == 'fast' else "🛡️ STABLE"
            self.get_logger().info(
                f"{mode_display} | Lap:{self.lap_count} | "
                f"Steer:{steering_angle:+.2f} | Speed:{target_speed:.1f}"
            )

    def visualize_goal(self, goal_coord):
        """Simple goal visualization"""
        if len(goal_coord) == 2:
            marker = Marker()
            marker.header.frame_id = "ego_racecar/base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.id = 0
            marker.pose.position.x = float(goal_coord[0])
            marker.pose.position.y = float(goal_coord[1])
            self.gap_viz_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    reactive_node = GapFollower()
    print("\n" + "="*70)
    print("🏁🏁🏁 TWO-PHASE RACING - FAST LAP THEN CRUISE 🏁🏁🏁")
    print("="*70)
    print(f"Phase 1 (Laps 1-{reactive_node.fast_laps}): TARGET 1:0x")
    print(f"Phase 2 (Lap {reactive_node.switch_mode_after}+): TARGET 1:30")
    print("="*70 + "\n")
    
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()