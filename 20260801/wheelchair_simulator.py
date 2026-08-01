import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
from geometry_msgs.msg import Twist, TransformStamped # importamos los mensajes de velocidad y de transformacion
from sensor_msgs.msg import LaserScan # importamos el mensaje de escaneo laser para el scan falso de collision_monitor
from tf2_ros import TransformBroadcaster # importamos el emisor de transformaciones para publicar la pose
import math # importamos math para los calculos del modelo cinematico

class WheelchairSimulator(Node): # definimos el nodo que simula el movimiento de la silla
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('wheelchair_simulator') # nombramos e inicializamos el nodo en la red de ros 2

        # estado interno: pose de la silla en el sistema de referencia odom
        self.x = 0.0 # posicion inicial en x
        self.y = 0.0 # posicion inicial en y
        self.theta = 0.0 # orientacion inicial en radianes

        # ultima velocidad recibida del controlador
        self.vx = 0.0 # velocidad lineal recibida
        self.wz = 0.0 # velocidad angular recibida

        self.last_time = self.get_clock().now() # guardamos el instante actual para integrar correctamente

        # nos suscribimos a la velocidad YA FILTRADA por velocity_smoother y collision_monitor (ver mi_configuracion.yaml).
        # /cmd_vel_nav es la salida CRUDA del controlador mppi, antes de suavizado y antes del chequeo de colision;
        # si el simulador escuchara ahi (como hacia antes), el limite de aceleracion y la parada de seguridad
        # (PolygonStop) quedarian configurados pero sin ningun efecto real sobre el movimiento simulado.
        self.create_subscription(Twist, '/cmd_vel_monitor_out', self.cmd_vel_callback, 10) # suscriptor de velocidades

        self.tf_broadcaster = TransformBroadcaster(self) # creamos el emisor de la transformacion odom -> base_link

        # bucle de integracion a 50 hz (mas fluido que la frecuencia del controlador)
        self.create_timer(0.02, self.update_pose) # temporizador que actualiza la pose cada 20 ms

        # publicador del scan "fantasma" que espera collision_monitor (ver mi_configuracion.yaml, dummy_scan).
        # sin ESTE topic, collision_monitor no tiene datos de ningun sensor y por seguridad bloquea TODA
        # velocidad (ver su log: "Robot to stop due to invalid source"), dejando la silla congelada aunque
        # el controlador si este generando comandos. Mientras no haya un laser real conectado, publicamos
        # un escaneo "siempre despejado" (360 grados sin retorno) para que collision_monitor tenga datos
        # frescos y solo actue cuando de verdad se le indique un obstaculo (p.ej. inyectandolo en las ranges).
        self.scan_publisher = self.create_publisher(LaserScan, '/scan_fantasma', 10) # publicador del scan falso
        self.create_timer(0.1, self.publish_fake_scan) # publicamos el scan falso a 10 hz

        self.get_logger().info("Wheelchair simulator started. Waiting for commands on /cmd_vel_monitor_out...") # avisamos que el simulador arranca

    def cmd_vel_callback(self, msg): # funcion que se ejecuta cuando llega una velocidad nueva
        self.vx = msg.linear.x # guardamos la velocidad lineal recibida
        self.wz = msg.angular.z # guardamos la velocidad angular recibida

    def publish_fake_scan(self): # funcion que publica un escaneo laser "siempre despejado" en /scan_fantasma
        scan = LaserScan() # creamos el mensaje de escaneo vacio
        scan.header.stamp = self.get_clock().now().to_msg() # marca de tiempo actual, para que no caduque por timeout
        scan.header.frame_id = 'base_link' # el scan esta centrado en la propia silla, sin necesitar tf adicional

        scan.angle_min = -math.pi # empezamos el barrido en -180 grados
        scan.angle_max = math.pi # terminamos el barrido en +180 grados
        scan.angle_increment = math.pi / 180.0 # un punto cada grado (360 puntos en total)
        scan.time_increment = 0.0 # asumimos que todos los puntos se toman en el mismo instante
        scan.scan_time = 0.1 # coincide con el periodo del temporizador (10 hz)
        scan.range_min = 0.05 # distancia minima que el sensor podria detectar
        scan.range_max = 10.0 # distancia maxima que el sensor podria detectar

        num_points = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) # cuantos puntos entran en el barrido
        # 'inf' es la convencion estandar de sensor_msgs/LaserScan para "sin retorno" (nada detectado dentro de rango),
        # asi collision_monitor recibe datos validos y frescos pero no ve ningun obstaculo real
        scan.ranges = [float('inf')] * num_points # rellenamos todo el barrido como despejado

        self.scan_publisher.publish(scan) # publicamos el scan falso

    def update_pose(self): # funcion que integra la velocidad para actualizar la pose y publicar la transformacion
        now = self.get_clock().now() # tomamos el instante actual
        dt = (now - self.last_time).nanoseconds / 1e9 # calculamos el tiempo transcurrido en segundos
        self.last_time = now # actualizamos la marca de tiempo para la siguiente iteracion

        # modelo cinematico diferencial: dx/dt = v*cos(theta), dy/dt = v*sin(theta), dtheta/dt = w
        self.x += self.vx * math.cos(self.theta) * dt # actualizamos la posicion en x
        self.y += self.vx * math.sin(self.theta) * dt # actualizamos la posicion en y
        self.theta += self.wz * dt # actualizamos la orientacion

        # publicamos la transformacion odom -> base_link
        t = TransformStamped() # creamos el mensaje de transformacion
        t.header.stamp = now.to_msg() # ponemos la marca de tiempo actual
        t.header.frame_id = 'odom' # el sistema de referencia padre es odom
        t.child_frame_id = 'base_link' # el sistema de referencia hijo es la base de la silla
        t.transform.translation.x = self.x # trasladamos en x segun la pose calculada
        t.transform.translation.y = self.y # trasladamos en y segun la pose calculada
        t.transform.translation.z = 0.0 # la altura es cero porque trabajamos en 2d
        t.transform.rotation.z = math.sin(self.theta / 2.0) # componente z del cuaternion de orientacion
        t.transform.rotation.w = math.cos(self.theta / 2.0) # componente w del cuaternion de orientacion

        self.tf_broadcaster.sendTransform(t) # publicamos la transformacion para que el resto del sistema vea la silla

def main(args=None): # funcion principal que arranca el nodo
    rclpy.init(args=args) # inicializamos las comunicaciones de ros 2
    node = WheelchairSimulator() # creamos una instancia del simulador
    try: # mantenemos el nodo vivo integrando el movimiento
        rclpy.spin(node) # cedemos el control a ros 2
    except KeyboardInterrupt: # si pulsamos ctrl+c en la terminal
        pass # no hacemos nada especial, solo salimos
    finally: # pase lo que pase liberamos los recursos
        node.destroy_node() # destruimos el nodo
        rclpy.shutdown() # cerramos las comunicaciones de ros 2

if __name__ == '__main__': # si ejecutamos este archivo directamente
    main() # llamamos a la funcion principal
