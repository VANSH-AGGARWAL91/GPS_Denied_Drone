#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time

class ZedToPx4Bridge(Node):
    def __init__(self):
        super().__init__('zed_px4_bridge')
        
        # Declare parameters
        self.declare_parameter('mavlink_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        
        port = self.get_parameter('mavlink_port').value
        baud = self.get_parameter('baud_rate').value
        
        # MAVLink connection
        self.get_logger().info(f'Connecting to MAVLink on {port} @ {baud}...')
        self.master = mavutil.mavlink_connection(port, baud=baud)
        self.master.wait_heartbeat()
        self.get_logger().info("✓ MAVLink heartbeat received!")
        
        # Subscribe to ZED pose
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Bridge node started. Waiting for ZED pose data...')
        self.msg_count = 0
        
    def pose_callback(self, msg):
        """
        Convert ZED camera frame to NED frame and send to PX4
        ZED: X=Right, Y=Down, Z=Forward
        NED: X=North, Y=East, Z=Down
        """
        
        # Frame transformation: ZED -> NED
        x_ned = msg.pose.position.z   # Forward becomes North
        y_ned = msg.pose.position.x   # Right becomes East
        z_ned = -msg.pose.position.y  # Down stays Down (negate for up)
        
        # Quaternion (orientation)
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        
        # Timestamp in microseconds
        time_usec = int(time.time() * 1e6)
        
        # Covariance matrix (21 elements)
        covariance = [0.01] * 21
        
        # CORRECT MAVLink function call - 8 arguments
        self.master.mav.vision_position_estimate_send(
            time_usec,   # uint64_t usec
            x_ned,       # float x
            y_ned,       # float y
            z_ned,       # float z
            0,           # float roll
            0,           # float pitch
            0,           # float yaw
            covariance   # float covariance[21] - optional 8th argument
        )
        
        self.msg_count += 1
        
        # Log every 50 messages to avoid spam
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f'✓ Sent {self.msg_count} msgs | Pos(NED): '
                f'x={x_ned:.2f}m, y={y_ned:.2f}m, z={z_ned:.2f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZedToPx4Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()