import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import TwistStamped

from pynput import keyboard

rtd = 180 / 3.1415
dtr = 1 / rtd


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.get_logger().info('Creando nodo keyboard_teleop')
        self.pub_velcmd = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.pub_diffcmd = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        # self.pub_wheel2 = self.create_publisher(Float64, '/commands/joint_wheel2', 10)
        # self.pub_wheel3 = self.create_publisher(Float64, '/commands/joint_wheel3', 10)
        # self.pub_arm1 = self.create_publisher(Float64, '/commands/joint_arm1', 10)
        # self.pub_arm2 = self.create_publisher(Float64, '/commands/joint_arm2', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.moveFw = False
        self.moveBw = False
        # self.strfLf = False
        # self.strfRt = False
        self.rotCw = False
        self.rotCcw = False

        self.defaultWheelSpeed = 6.0
        self.defaultLinearSpeed = 0.5
        self.defaultAngularSpeed = 0.5

    def on_press(self, key):
        try:
            # self.get_logger().info('alphanumeric key {0} pressed'.format(key.char))
            match key.char:
                case 'w':
                    self.moveFw = True
                case 's':
                    self.moveBw = True
                case 'a':
                    self.rotCcw = True
                case 'd':
                    self.rotCw = True
                case _:
                    self.get_logger().warn('Undefined key command: {0}'.format(key))
        except AttributeError:
            self.get_logger().debug('special key {0} pressed'.format(key))

    def on_release(self, key):
        try:
            match key.char:
                case 'w':
                    self.moveFw = False
                case 's':
                    self.moveBw = False
                case 'a':
                    self.rotCcw = False
                case 'd':
                    self.rotCw = False
        except AttributeError:
            self.get_logger().debug('special key {0} pressed'.format(key))

    def timer_callback(self):
        # GET KEYS
        msg_velcmd = Float64MultiArray()

        msg_velcmd.data = [self.defaultWheelSpeed * (- self.moveFw + self.moveBw + (self.rotCcw - self.rotCw) / 2.0),
                           self.defaultWheelSpeed * (- self.moveFw + self.moveBw - (self.rotCcw - self.rotCw) / 2.0),
                           ]

        msg_diffcmd = TwistStamped()
        msg_diffcmd.header.stamp = self.get_clock().now().to_msg()
        msg_diffcmd.twist.linear.x = self.defaultLinearSpeed * (self.moveFw - self.moveBw)
        msg_diffcmd.twist.angular.z = self.defaultAngularSpeed * (self.rotCcw - self.rotCw)

        # self.pub_velcmd.publish(msg_velcmd)
        self.pub_diffcmd.publish(msg_diffcmd)


def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop = KeyboardTeleop()
    rclpy.spin(keyboard_teleop)

    keyboard_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
