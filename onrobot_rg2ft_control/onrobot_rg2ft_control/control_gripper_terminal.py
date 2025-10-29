#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket

class GripperTerminal(Node):
    def __init__(self):
        super().__init__('gripper_terminal')
        self.gripper_ip = "192.168.1.1"
        self.gripper_port = 502
        self.sock = None

        self.connect_to_gripper()
        self.get_logger().info("Escribe 'a' para abrir, 'c' para cerrar, 'q' para salir")
        self.run_loop()

    def connect_to_gripper(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2)
            self.sock.connect((self.gripper_ip, self.gripper_port))
            self.get_logger().info(f"✅ Conectado al gripper en {self.gripper_ip}:{self.gripper_port}")
        except Exception as e:
            self.get_logger().error(f"No se pudo conectar al gripper: {e}")

    def send_command(self, cmd_bytes):
        try:
            self.sock.send(cmd_bytes)
            resp = self.sock.recv(64)
            self.get_logger().info(f"📩 Respuesta: {resp.hex()}")
        except Exception as e:
            self.get_logger().error(f"Error enviando comando: {e}")

    def open_gripper(self):
        cmd = bytes.fromhex("0001000000060106059E03E8")  # abrir
        self.send_command(cmd)
        self.get_logger().info("🟢 Gripper abriendo...")

    def close_gripper(self):
        cmd = bytes.fromhex("0002000000060106059E0000")  # cerrar
        self.send_command(cmd)
        self.get_logger().info("🔴 Gripper cerrando...")

    def run_loop(self):
        while rclpy.ok():
            try:
                user_input = input("> ").strip().lower()
                if user_input == "a":
                    self.open_gripper()
                elif user_input == "c":
                    self.close_gripper()
                elif user_input == "q":
                    self.get_logger().info("👋 Saliendo del nodo...")
                    break
                else:
                    print("Usa 'a' para abrir, 'c' para cerrar, 'q' para salir")
            except KeyboardInterrupt:
                break
        self.sock.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperTerminal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

