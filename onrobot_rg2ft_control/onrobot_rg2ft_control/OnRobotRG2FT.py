#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg2ft_control.OnRobotTcpClient import OnRobotTcpClient
from onrobot_rg2ft_msgs.msg import RG2FTCommand, OnRobotRG2FTData
from onrobot_rg2ft_msgs.srv import SetProximityOffsets


import struct

RG2FT_MIN_WIDTH = 0
RG2FT_MAX_WIDTH = 1000
RG2FT_MIN_FORCE = 0
RG2FT_MAX_FORCE = 400
RG2FT_DEVICE_ADDRESS = 65

def s16(val):
    return struct.unpack('h', struct.pack('H', val))[0]

def u16(val):
    return struct.unpack('H', struct.pack('h', val))[0]

class OnRobotRG2FTNode(Node):

    def __init__(self):
        super().__init__('onrobot_rg2ft_node')

        # Parámetros
        self.declare_parameter('ip', '192.168.1.1')
        self.declare_parameter('port', 502)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Cliente TCP
        self.client = OnRobotTcpClient()
        self.client.connect(ip, port, RG2FT_DEVICE_ADDRESS)

        # Publicador del estado
        self.state_pub = self.create_publisher(OnRobotRG2FTData, 'gripper/state', 10)

        # Suscriptor de comandos
        self.cmd_sub = self.create_subscription(RG2FTCommand, 'gripper/command', self.cmd_callback, 10)

        # Timer para publicar estado periódicamente
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz

    def cmd_callback(self, cmd: RG2FTCommand):
        if cmd.target_width < RG2FT_MIN_WIDTH or cmd.target_width > RG2FT_MAX_WIDTH:
            self.get_logger().error(f"Target width {cmd.target_width} out of range")
            return

        if cmd.target_force < RG2FT_MIN_FORCE or cmd.target_force > RG2FT_MAX_FORCE:
            self.get_logger().error(f"Target force {cmd.target_force} out of range")
            return

        if cmd.control not in [0, 1]:
            self.get_logger().error(f"Control value {cmd.control} not valid")
            return

        message = [cmd.target_force, cmd.target_width, cmd.control]
        self.client.write(address=2, message=message)

    def read_state(self) -> OnRobotRG2FTData:
	    resp = self.client.read(address=257, count=26)
	    msg = OnRobotRG2FTData()
	    msg.status_l = resp[0]
	    msg.fx_l = s16(resp[2])
	    msg.fy_l = s16(resp[3])
	    msg.fz_l = s16(resp[4])
	    msg.tx_l = s16(resp[5])
	    msg.ty_l = s16(resp[6])
	    msg.tz_l = s16(resp[7])
	    msg.status_r = resp[9]
	    msg.fx_r = s16(resp[11])
	    msg.fy_r = s16(resp[12])
	    msg.fz_r = s16(resp[13])
	    msg.tx_r = s16(resp[14])
	    msg.ty_r = s16(resp[15])
	    msg.tz_r = s16(resp[16])
	    msg.proximity_status_l = resp[17]
	    msg.proximity_value_l = s16(resp[18])
	    msg.proximity_status_r = resp[20]
	    msg.proximity_value_r = s16(resp[21])
	    msg.actual_gripper_width = s16(resp[23])
	    msg.gripper_busy = resp[24]
	    msg.grip_detected = resp[25]
	    return msg



    def publish_state(self):
        state_msg = self.read_state()
        self.state_pub.publish(state_msg)

    def set_proximity_offsets(self, left_offset: int, right_offset: int):
        message = [left_offset, right_offset]
        self.client.write(address=5, message=message)

    def zero_force_torque(self, val: int):
        self.client.write(address=0, message=[val])

    def restart_power_cycle(self):
        self.client.restartPowerCycle()


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRG2FTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

