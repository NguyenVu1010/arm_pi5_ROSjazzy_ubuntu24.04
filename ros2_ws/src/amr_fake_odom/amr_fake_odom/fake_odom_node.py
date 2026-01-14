import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.vx = 0.0; self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.update)

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.x += (self.vx * math.cos(self.th)) * dt
        self.y += (self.vx * math.sin(self.th)) * dt
        self.th += self.vth * dt

        q = self.euler_to_quaternion(0, 0, self.th)
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_br.sendTransform(t)

        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = q
        self.odom_pub.publish(o)

    def euler_to_quaternion(self, r, p, y):
        qx = math.sin(r/2)*math.cos(p/2)*math.cos(y/2) - math.cos(r/2)*math.sin(p/2)*math.sin(y/2)
        qy = math.cos(r/2)*math.sin(p/2)*math.cos(y/2) + math.sin(r/2)*math.cos(p/2)*math.sin(y/2)
        qz = math.cos(r/2)*math.cos(p/2)*math.sin(y/2) - math.sin(r/2)*math.sin(p/2)*math.cos(y/2)
        qw = math.cos(r/2)*math.cos(p/2)*math.cos(y/2) + math.sin(r/2)*math.sin(p/2)*math.sin(y/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeOdomNode())
    rclpy.shutdown()