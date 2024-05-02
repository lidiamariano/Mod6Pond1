#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill


class Desenhador(Node):
    def __init__(self):
        super().__init__('desenhador')  # Fornecendo um nome para o nó
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        pass

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        self.publisher.publish(msg)

        self.get_logger().info('Movendo para frente...')
        time.sleep(0.5)

        msg.linear.x = 0.0
        self.publisher.publish(msg)

        msg.angular.z = 1.0
        self.publisher.publish(msg)

        self.get_logger().info('Desenhando um círculo...')
        time.sleep(0.5)

        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def spawn_turtle(self, x, y, theta):
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de spawn não disponível, tentando novamente...')
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = 'new_turtle'
        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Tartaruga spawnada com sucesso: %s' % future.result().name)
        else:
            self.get_logger().info('Falha ao spawnar tartaruga')

    def kill_turtle(self, name):
        kill_client = self.create_client(Kill, 'kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de kill não disponível, tentando novamente...')
        request = Kill.Request()
        request.name = name
        future = kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Tartaruga %s morta com sucesso' % name)
        else:
            self.get_logger().info('Falha ao matar tartaruga %s' % name)


def main(args=None):
    rclpy.init(args=args)

    desenhador = Desenhador()

    desenhador.spawn_turtle(5.0, 5.0, 0.0)
    time.sleep(1.0)
    desenhador.kill_turtle('new_turtle')

    rclpy.spin(desenhador)

    desenhador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
