import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
import serial
import math

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_sender')

        self.get_logger().info(f'{self.get_parameter("use_sim_time").value}')

        self.serial_port = '/dev/ttyUSB0'

        self.baud_rate = 115200
        self.joint_pub = self.create_publisher(JointState,"joint_states", 10) 
        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial connected to {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_commands',
            self.listener_callback,
            10
        )

        self.timer_pub = self.create_timer(0.01, self.ler_serial) #100vezes/seg

    def listener_callback(self, msg):
        # O nó C++ envia [pid_esquerdo, pid_direito]
        if len(msg.data) >= 2:
            pid_l = msg.data[0]
            pid_r = msg.data[1]
            
            message = f"{pid_l:.2f} {pid_r:.2f}\n"
            
            try:
                self.ser.write(message.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")

    def ler_serial(self):
        if self.ser.in_waiting > 0:  # Check if there is data in the buffer
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            self.get_logger().info("Received: "+ line)
            self.publicar(line)

    def publicar(self, linha_lida:str):  # mudar
        array = linha_lida.split()
        if(len(array)==5):
            try:
                self.pos_mot_m1 = float(array[0])
                self.pos_mot_m2 = float(array[1])
                self.pos_rad_mot_m1 = self.pos_mot_m1 * 2 * math.pi
                self.pos_rad_mot_m2 = self.pos_mot_m2 * 2 * math.pi

                msg = JointState()
                now = self.get_clock().now().to_msg()
                msg.header.stamp = now
                msg.name = ["wheel_left_joint", "wheel_right_joint"]
                msg.position = [self.pos_rad_mot_m2, self.pos_rad_mot_m1]  # Cuidado com a ordem
                msg.velocity = []
                msg.effort = []
                self.joint_pub.publish(msg)

                msg_imu = Imu()
                msg_imu.header.stamp = now
                msg_imu.linear_acceleration.x = float(array[2])
                msg_imu.linear_acceleration.y = float(array[3])
                msg_imu.angular_velocity.z = float(array[4])
                self.imu_pub.publish(msg_imu)
            except ValueError:
                self.get_logger().info("Erro na leitura serial")



def main(args=None):
    rclpy.init(args=args)
    node = SerialController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()