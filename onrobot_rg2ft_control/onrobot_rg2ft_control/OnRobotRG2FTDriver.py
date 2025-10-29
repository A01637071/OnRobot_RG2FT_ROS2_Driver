#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from onrobot_rg2ft_control.OnRobotRG2FT import OnRobotRG2FTNode
from std_srvs.srv import Trigger, SetBool
from onrobot_rg2ft_msgs.msg import RG2FTCommand, OnRobotRG2FTData
from geometry_msgs.msg import Wrench
from onrobot_rg2ft_msgs.srv import SetProximityOffsets



class OnRobotRG2FTDriver(Node):
    def __init__(self):
        super().__init__('onrobot_rg2ft_driver')

        # Parámetros configurables (IP y puerto)
        self.declare_parameter('ip', '192.168.1.1')
        self.declare_parameter('port', 502)
        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value

        # Conexión con el gripper
        self.get_logger().info(f'Conectando al gripper en {ip}:{port} ...')
        self.gripper = OnRobotRG2FTNode()
        self.get_logger().info('✅ Gripper conectado correctamente')

        # Publishers
        self.state_pub = self.create_publisher(OnRobotRG2FTData, 'state', 10)
        self.left_wrench_pub = self.create_publisher(Wrench, 'left_wrench', 10)
        self.right_wrench_pub = self.create_publisher(Wrench, 'right_wrench', 10)

        # Subscriber de comandos
        self.cmd_sub = self.create_subscription(
            RG2FTCommand, 'gripper/command', self.command_callback, 10
        )

        # Servicios
        self.restart_srv = self.create_service(Trigger, 'restart', self.restart_cb)
        self.zero_srv = self.create_service(SetBool, 'zero_force_torque', self.zero_force_torque_cb)
        self.set_prox_offset_srv = self.create_service(
    		SetProximityOffsets,
    		'set_proximity_offsets',
    		self.prox_offsets_cb
	)


        # Timer para publicar estado periódicamente
        self.timer = self.create_timer(0.01, self.publish_state)  # 100 Hz

    # ---------- CALLBACKS ----------
    def command_callback(self, msg):
        self.get_logger().info('Recibido comando para el gripper.')
        try:
            self.gripper.cmd_callback(msg)
        except Exception as e:
            self.get_logger().error(f'Error al enviar comando al gripper: {e}')



    def restart_cb(self, request, response):
        self.get_logger().info('Reiniciando alimentación del gripper...')
        self.gripper.restartPowerCycle()
        response.success = True
        response.message = 'Power cycle completo.'
        return response

    def zero_force_torque_cb(self, request, response):
        self.get_logger().info('Reiniciando valores de fuerza/par.')
        self.gripper.zeroForceTorque(request.data)
        response.success = True
        response.message = 'Offsets de fuerza/par reiniciados.'
        return response

    def prox_offsets_cb(self, request, response):
    	self.get_logger().info('Configurando offsets de proximidad...')

    # Enviar offsets al gripper
    	self.gripper.set_proximity_offsets(
        	request.proximity_offset_l,
        	request.proximity_offset_r
    	)

    	response.success = True
    	response.message = (
        	f"Offsets configurados: izquierda={request.proximity_offset_l}, "
        	f"derecha={request.proximity_offset_r}"
    	)
    	return response


    def publish_state(self):
    	try:
        	state = self.gripper.read_state()
        	#self.get_logger().info(f"Estado leído: width={state.actual_gripper_width}, 		FxL={state.fx_l}")
        	self.state_pub.publish(state)
        	self.publish_wrenches(state)
    	except Exception as e:
        	self.get_logger().warn(f'Error al leer estado: {e}')


    def publish_wrenches(self, state):
        left_wrench = Wrench()
        left_wrench.force.x = state.fx_l / 10
        left_wrench.force.y = state.fy_l / 10
        left_wrench.force.z = state.fz_l / 10
        left_wrench.torque.x = state.tx_l / 100
        left_wrench.torque.y = state.ty_l / 100
        left_wrench.torque.z = state.tz_l / 100

        right_wrench = Wrench()
        right_wrench.force.x = state.fx_r / 10
        right_wrench.force.y = state.fy_r / 10
        right_wrench.force.z = state.fz_r / 10
        right_wrench.torque.x = state.tx_r / 100
        right_wrench.torque.y = state.ty_r / 100
        right_wrench.torque.z = state.tz_r / 100

        self.left_wrench_pub.publish(left_wrench)
        self.right_wrench_pub.publish(right_wrench)


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRG2FTDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        if rclpy.ok():
        	rclpy.shutdown()


if __name__ == '__main__':
    main()

