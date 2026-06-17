import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
from nav_msgs.msg import OccupancyGrid, MapMetaData # importamos los mensajes para publicar mapas de ocupacion
from std_msgs.msg import Header # importamos la cabecera estandar para marcar tiempo y sistema de referencia
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy # importamos las clases para configurar la calidad de servicio
import numpy as np # importamos numpy para manipular la matriz de costes
from pathlib import Path # importamos Path para construir rutas robustas relativas al archivo

from read_environment_json import load_environment_json # importamos la funcion que lee el json del entorno
from cost_module import load_cost_library, complex_cost_injection # importamos las funciones que cargan costes y generan el mapa

class CostInjectionNode(Node): # definimos el nodo encargado de generar y publicar el mapa de costes
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('cost_injection_node') # nombramos e inicializamos el nodo para que ros 2 lo reconozca en la red

        qos_profile = QoSProfile( # creamos un perfil de calidad de servicio compatible con nav2
            depth=1, # guardamos solo el ultimo mensaje en la cola
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # hacemos que los suscriptores tardios reciban el ultimo mapa
            reliability=QoSReliabilityPolicy.RELIABLE # garantizamos la entrega fiable de los mensajes
        )

        self.publisher_ = self.create_publisher(OccupancyGrid, '/mapa_de_costes', qos_profile) # creamos el publicador del mapa de costes con el qos definido

        timer_period = 2.0 # definimos que publicaremos un mapa cada 2 segundos
        self.timer = self.create_timer(timer_period, self.publish_map) # creamos un temporizador que llama a la funcion de publicacion

        self.get_logger().info('Starting cost injection') # avisamos por terminal que el nodo arranca
        self.base_dir = Path(__file__).resolve().parent # obtenemos la carpeta real donde vive este script
        self.cost_library = load_cost_library(self.base_dir / 'costes_preestablecidos.json') # cargamos la biblioteca de costes desde el json

        self.detected_objects = load_environment_json(self.base_dir / 'ejemplo_natalia.json') # cargamos el json del entorno con habitaciones y objetos

        self.costmap = None # inicializamos la matriz de costes vacia hasta que se genere
        if self.detected_objects and self.cost_library: # comprobamos que se han cargado correctamente los datos
            # recibimos la matriz y todas las variables matematicas de la bounding box
            self.costmap, self.map_w, self.map_h, self.origin_x, self.origin_y = complex_cost_injection(self.detected_objects, self.cost_library) # generamos el mapa de costes
            self.get_logger().info('Matrix successfully generated. Starting publication') # avisamos que la matriz ya esta lista
        else: # si faltan datos no podemos generar el mapa
            self.get_logger().error('Missing data. Check the JSON files.') # informamos del error por terminal

    def publish_map(self): # funcion que se ejecuta cada 2 segundos para publicar el mapa
        if self.costmap is None: # si todavia no hay matriz generada
            return # no hacemos nada y salimos

        msg = OccupancyGrid() # creamos el mensaje del mapa de ocupacion

        msg.header = Header() # preparamos la cabecera del mensaje
        msg.header.stamp = self.get_clock().now().to_msg() # ponemos la marca de tiempo actual
        msg.header.frame_id = 'map' # anclamos el mapa al sistema de referencia del mundo real

        msg.info = MapMetaData() # preparamos los metadatos del mapa
        msg.info.resolution = 0.1 # indicamos que cada celda mide 10 cm

        # el tamano en celdas es el ancho en metros dividido entre la resolucion
        msg.info.width = int(self.map_w / msg.info.resolution) # calculamos el ancho del mapa en celdas
        msg.info.height = int(self.map_h / msg.info.resolution) # calculamos el alto del mapa en celdas

        # colocamos el origen justo en la esquina de nuestra bounding box
        msg.info.origin.position.x = float(self.origin_x) # fijamos la coordenada x del origen del mapa
        msg.info.origin.position.y = float(self.origin_y) # fijamos la coordenada y del origen del mapa
        msg.info.origin.position.z = 0.0 # la altura del origen es cero porque trabajamos en 2d

        # =================================================================
        # NORMALIZACION DE COSTES A ESCALA NAV2 (vectorizada y explicita)
        # =================================================================
        # escala interna (uint8): 0 = libre, 1-252 = costes graduales, 253 = inflado, 254 = letal
        # escala nav2 occupancygrid: 0 = libre, 1-99 = costes graduales, 100 = letal
        data = self.costmap.astype(np.int16) # convertimos la matriz a int16 para tener margen durante los calculos
        output = np.empty_like(data) # creamos una matriz vacia del mismo tamano para guardar el resultado

        # anclamos los valores frontera de forma explicita para no perder la distincion entre letal e inflado
        output[data == 0] = 0 # las celdas libres se quedan en 0
        output[data == 254] = 100 # el obstaculo letal se convierte en 100
        output[data == 253] = 99 # la inflacion maxima se convierte en 99

        # para el resto (1..252) escalamos linealmente al rango 1..98 usando redondeo para no perder costes bajos
        mask = (data > 0) & (data < 253) # seleccionamos las celdas con coste intermedio
        output[mask] = np.clip( # recortamos el resultado para que se quede dentro del rango permitido
            np.round(data[mask] * 98.0 / 252.0).astype(np.int16), # escalamos y redondeamos los costes intermedios
            1, 98 # limitamos los valores entre 1 y 98
        )

        msg.data = output.flatten().tolist() # aplanamos la matriz a una lista de una dimension que entiende ros 2
        self.publisher_.publish(msg) # publicamos el mapa de costes en el topic
        self.get_logger().info('Publishing cost map') # avisamos por terminal de cada publicacion

def main(args=None): # funcion principal que arranca el nodo
    rclpy.init(args=args) # inicializamos las comunicaciones de ros 2

    node = CostInjectionNode() # creamos una instancia de nuestro nodo

    try: # mantenemos el nodo vivo publicando mapas
        rclpy.spin(node) # cedemos el control a ros 2 para que procese los temporizadores
    except KeyboardInterrupt: # si pulsamos ctrl+c en la terminal
        node.get_logger().info('Shutting down the injection node') # avisamos que vamos a apagar el nodo
    finally: # pase lo que pase limpiamos los recursos
        node.destroy_node() # destruimos el nodo y sus componentes
        rclpy.shutdown() # cerramos las comunicaciones de ros 2

if __name__ == '__main__': # si ejecutamos este archivo directamente
    main() # llamamos a la funcion principal
