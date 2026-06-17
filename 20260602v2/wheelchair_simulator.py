import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
from geometry_msgs.msg import Twist, TransformStamped # importamos los mensajes de velocidad y de transformacion
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

        # nos suscribimos al canal donde el controlador mppi publica las velocidades
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10) # suscriptor de velocidades

        self.tf_broadcaster = TransformBroadcaster(self) # creamos el emisor de la transformacion odom -> base_link

        # bucle de integracion a 50 hz (mas fluido que la frecuencia del controlador)
        self.create_timer(0.02, self.update_pose) # temporizador que actualiza la pose cada 20 ms

        self.get_logger().info("Wheelchair simulator started. Waiting for commands on /cmd_vel_nav...") # avisamos que el simulador arranca

    def cmd_vel_callback(self, msg): # funcion que se ejecuta cuando llega una velocidad nueva
        self.vx = msg.linear.x # guardamos la velocidad lineal recibida
        self.wz = msg.angular.z # guardamos la velocidad angular recibida

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
