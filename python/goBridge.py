#!/usr/bin/env python3
#
# This node bridges a Go backend (via WebSocket) to ROS 2.
# It listens for JSON messages and publishes them to the
# correct topics for Cartographer.
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations # pip install tf-transformations
import json
import asyncio
import websockets
import math

class GoBridgeNode(Node):
    def __init__(self):
        super().__init__('go_bridge_node')
        
        # --- Publishers ---
        # These topic names MUST match your Cartographer config
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)

        # --- Frame IDs ---
        # These MUST match your TF tree (URDF, static transforms, etc.)
        self.odom_frame_id = 'odom'
        self.base_frame_id = 'base_link'
        self.laser_frame_id = 'laser_frame'
        self.imu_frame_id = 'imu_link'

        self.get_logger().info('Go Bridge Node has started.')
        self.get_logger().info('Listening for Odom, IMU, and Scan data...')

    def publish_scan(self, data):
        """ Publishes a sensor_msgs/LaserScan message """
        try:
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.laser_frame_id
            
            # Populate the message from the JSON data
            msg.angle_min = data['angle_min']
            msg.angle_max = data['angle_max']
            msg.angle_increment = data['angle_increment']
            msg.range_min = data['range_min']
            msg.range_max = data['range_max']
            # Ensure ranges are floats
            msg.ranges = [float(r) for r in data['ranges']]
            
            self.scan_publisher.publish(msg)
            # self.get_logger().info('Published LaserScan.', throttle_duration_sec=5.0)
        except KeyError as e:
            self.get_logger().error(f'Malformed "scan" JSON: missing key {e}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_scan: {e}')

    def publish_odom(self, data):
        """ Publishes a nav_msgs/Odometry message """
        try:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.odom_frame_id
            msg.child_frame_id = self.base_frame_id

            # --- Pose ---
            msg.pose.pose.position.x = data['x']
            msg.pose.pose.position.y = data['y']
            msg.pose.pose.position.z = 0.0 # Assuming 2D
            
            # Convert yaw (theta) to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, data['theta'])
            msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # --- Twist (Velocity) ---
            # Uncomment if your Go code sends velocity
            # msg.twist.twist.linear.x = data.get('vx', 0.0)
            # msg.twist.twist.angular.z = data.get('vtheta', 0.0)

            self.odom_publisher.publish(msg)
            # self.get_logger().info('Published Odometry.', throttle_duration_sec=5.0)
        except KeyError as e:
            self.get_logger().error(f'Malformed "odom" JSON: missing key {e}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_odom: {e}')

    def publish_imu(self, data):
        """ Publishes a sensor_msgs/Imu message """
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.imu_frame_id
            
            # Cartographer in 2D mainly uses angular_velocity.z and linear_acceleration
            msg.angular_velocity.x = data.get('gyro_x', 0.0)
            msg.angular_velocity.y = data.get('gyro_y', 0.0)
            msg.angular_velocity.z = data.get('gyro_z', 0.0)
            
            msg.linear_acceleration.x = data.get('accel_x', 0.0)
            msg.linear_acceleration.y = data.get('accel_y', 0.0)
            msg.linear_acceleration.z = data.get('accel_z', 0.0)
            
            # You must provide non-zero covariance or Cartographer may complain
            # Setting a small variance for measured axes
            msg.linear_acceleration_covariance[0] = 0.1
            msg.linear_acceleration_covariance[4] = 0.1
            msg.linear_acceleration_covariance[8] = 0.1
            
            msg.angular_velocity_covariance[0] = 0.1
            msg.angular_velocity_covariance[4] = 0.1
            msg.angular_velocity_covariance[8] = 0.1

            # We are not providing orientation in the IMU message
            msg.orientation_covariance[0] = -1.0 # Mark as not available

            self.imu_publisher.publish(msg)
            # self.get_logger().info('Published IMU.', throttle_duration_sec=5.0)
        except KeyError as e:
            self.get_logger().error(f'Malformed "imu" JSON: missing key {e}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_imu: {e}')


async def websocket_handler(websocket, path, ros_node):
    """ Handles incoming WebSocket messages """
    ros_node.get_logger().info('WebSocket client connected.')
    async for message in websocket:
        try:
            # Parse the wrapper message
            wrapper = json.loads(message)
            msg_type = wrapper.get('Type')
            msg_data = wrapper.get('Data')

            if not msg_type or msg_data is None:
                ros_node.get_logger().warn('Received message without Type or Data.')
                continue
                
            # Route based on type
            if msg_type == 'odom':
                ros_node.publish_odom(msg_data)
            elif msg_type == 'imu':
                ros_node.publish_imu(msg_data)
            elif msg_type == 'scan':
                ros_node.publish_scan(msg_data)
            else:
                ros_node.get_logger().warn(f'Received unknown message type: {msg_type}')

        except json.JSONDecodeError:
            ros_node.get_logger().error('Received malformed JSON.')
        except Exception as e:
            ros_node.get_logger().error(f'Error processing message: {e}')
    ros_node.get_logger().info('WebSocket client disconnected.')


async def main_async(args=None):
    """ Main function to run ROS node and WebSocket server """
    rclpy.init(args=args)
    go_bridge_node = GoBridgeNode()
    
    # Run the WebSocket server
    # Binds to 0.0.0.0 to be accessible from outside the WSL container
    # (e.g., from host Windows)
    start_server = websockets.serve(
        lambda ws, path: websocket_handler(ws, path, go_bridge_node), "0.0.0.0", 8765
    )
    
    # Run the WebSocket server and the ROS node concurrently
    try:
        # Run the websocket server
        await start_server
        # Spin the ROS node
        while rclpy.ok():
            rclpy.spin_once(go_bridge_node, timeout_sec=0.01)
            await asyncio.sleep(0.001) # Yield control
    except KeyboardInterrupt:
        pass
    finally:
        go_bridge_node.get_logger().info('Shutting down...')
        go_bridge_node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    asyncio.run(main_async(args=args))

if __name__ == '__main__':
    main()