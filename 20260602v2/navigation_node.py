import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
from rclpy.action import ActionClient # importamos el cliente de acciones para comunicarnos con nav2
import json # importamos json para procesar las respuestas del modelo de lenguaje
import requests # importamos requests para hacer peticiones http al servidor de ollama
import math # importamos math para convertir angulos en cuaterniones
from pathlib import Path # importamos Path para construir rutas robustas relativas al proyecto

from nav2_msgs.action import NavigateThroughPoses # importamos la accion de nav2 para navegar por varios puntos seguidos
from geometry_msgs.msg import PoseStamped # importamos el mensaje de pose que usa ros 2 para las coordenadas
from std_msgs.msg import String # importamos el mensaje de texto para recibir las ordenes del usuario

class SemanticNavigator(Node): # definimos el nodo que traduce ordenes habladas en navegacion
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('semantic_navigator_node') # nombramos e inicializamos el nodo en la red de ros 2

        self.get_logger().info("Starting the Semantic Navigation Node...") # avisamos por terminal que el nodo arranca

        # cola de destinos pendientes: actua como la memoria de la silla, guarda los sitios que aun debe visitar
        self.pending_goals = [] # lista de destinos en espera (cada uno es una tupla x, y, orientacion)
        self.meta_actual = None # manejador del destino que la silla tiene ahora mismo en curso (None si esta parada)
        self.current_waypoint = None # destino (x, y, orientacion) al que corresponde meta_actual, para poder reencolarlo si se interrumpe

        # palabras que, si aparecen en la orden, indican que el usuario quiere descartar lo que habia pendiente
        self.discard_keywords = ["quedate", "quédate", "solo", "solamente", "unicamente", "únicamente",
                                 "cancela", "olvida", "anula", "descarta", "ya no", "no vayas"] # disparadores del modo descartar

        # cargamos el mapa arquitectonico desde el json usando una ruta relativa a este archivo
        self.map_filepath = Path(__file__).resolve().parent / 'ejemplo_natalia.json' # ruta del json del entorno
        self.room_data = self.load_map_data(self.map_filepath) # construimos el diccionario de habitaciones y coordenadas

        # configuramos el punto de acceso de la api de ollama que corre en local
        self.ollama_url = "http://localhost:11434/api/generate" # direccion del servidor local del modelo de lenguaje
        self.llm_model = "phi3" # modelo que usamos, se puede cambiar por llama3 segun lo instalado en la zimaboard

        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses') # creamos el cliente de accion para enviar objetivos a nav2

        # creamos el suscriptor para recibir las ordenes del usuario en tiempo real
        self.command_subscriber = self.create_subscription( # nos suscribimos al topic de comandos
            String, # tipo de mensaje que esperamos (texto)
            '/comando_usuario', # nombre del topic por el que llegan las ordenes
            self.command_callback, # funcion que se ejecuta cuando llega un mensaje
            10 # tamano de la cola por si llegan varios mensajes seguidos
        )

        self.get_logger().info("Semantic Navigator ready! Waiting for commands on /comando_usuario...") # avisamos que el nodo esta listo

    def load_map_data(self, filepath): # funcion que lee el json y relaciona cada habitacion con su centro y orientacion
        try: # intentamos abrir y procesar el archivo de forma segura
            with open(filepath, 'r', encoding='utf-8') as f: # abrimos el json del entorno en modo lectura
                data = json.load(f) # cargamos el contenido del archivo

            room_centers = {} # creamos un diccionario vacio para guardar las coordenadas de cada habitacion
            for room in data.get('rooms', []): # recorremos todas las habitaciones del mapa
                name = room['name'].lower() # leemos el nombre de la habitacion en minusculas

                # calculamos el centro exacto de la habitacion
                center_x = room['position_x'] + (room['width'] / 2.0) # centro en x sumando media anchura a la esquina
                center_y = room['position_y'] + (room['height'] / 2.0) # centro en y sumando media altura a la esquina

                orientation = room.get('orientation', 0) # leemos la orientacion del json, o 0 si no existe

                room_centers[name] = (center_x, center_y, orientation) # guardamos los tres valores: x, y y orientacion

            self.get_logger().info(f"Loaded {len(room_centers)} rooms from the map.") # avisamos cuantas habitaciones hemos cargado
            return room_centers # devolvemos el diccionario de habitaciones

        except Exception as e: # si falla la lectura del mapa capturamos el error
            self.get_logger().error(f"Failed to load the map JSON: {e}") # informamos del problema por terminal
            return {} # devolvemos un diccionario vacio para que el nodo no se rompa

    def extract_rooms_from_prompt(self, user_text): # funcion que pide al modelo que extraiga las habitaciones de la frase
        # le decimos al modelo exactamente que habitaciones existen para evitar alucinaciones
        available_rooms = list(self.room_data.keys()) # obtenemos la lista de nombres de habitaciones validas

        # el system prompt define las reglas estrictas de comportamiento del modelo (se mantiene en espanol)
        system_prompt = ( # construimos las instrucciones del sistema para el modelo
            "Eres un analizador de datos estricto para un robot. "
            "Extrae de la frase del usuario UNICAMENTE los nombres de las habitaciones a las que el usuario SI quiere ir, en orden cronologico. "
            f"LISTA ESTRICTA DE HABITACIONES PERMITIDAS: {available_rooms}. "
            "REGLAS OBLIGATORIAS: "
            "1. NO traduzcas al ingles. Manten los nombres exactamente como estan en la lista en espanol. "
            "2. IGNORA los objetos (gafas, llaves, etc.) y las acciones. Solo me importan las habitaciones. "
            "3. REGLA DE NEGACION: si el usuario dice que NO quiere ir a una habitacion (por ejemplo 'ya no vayas a', 'no quiero ir a'), NO incluyas esa habitacion en la lista. "
            "4. Si una habitacion no esta en la lista permitida, NO la incluyas. "
            "5. Devuelve SOLO un array JSON de strings con las habitaciones a las que SI hay que ir."
        )

        payload = { # preparamos los datos que enviaremos al modelo
            "model": self.llm_model, # indicamos que modelo debe usar
            "prompt": f"Comando del usuario: '{user_text}'\n\nSalida esperada:", # le pasamos la frase del usuario
            "system": system_prompt, # adjuntamos las reglas del sistema
            "stream": False, # pedimos la respuesta completa de golpe, no en trozos
            "format": "json" # obligamos a ollama a devolver un json valido
        }

        self.get_logger().info(f"Asking the LLM to process: '{user_text}'...") # avisamos que estamos consultando al modelo

        try: # intentamos la comunicacion con el modelo de forma segura
            # enviamos la peticion http al servidor de ollama con 60 segundos de paciencia
            response = requests.post(self.ollama_url, json=payload, timeout=60.0) # lanzamos la peticion al servidor local
            response.raise_for_status() # comprobamos que la respuesta http es correcta

            result_text = response.json().get("response", "[]") # extraemos el texto de la respuesta del modelo

            parsed_json = json.loads(result_text) # convertimos ese texto json en una estructura de python

            # logica de robustez: rescatamos la lista venga como venga del modelo
            room_sequence = [] # inicializamos la secuencia de habitaciones vacia
            if isinstance(parsed_json, dict): # si el modelo devuelve un diccionario que envuelve la lista
                for key, value in parsed_json.items(): # recorremos las claves del diccionario
                    if isinstance(value, list): # si encontramos un valor que es una lista
                        room_sequence = value # nos quedamos con esa lista
                        break # dejamos de buscar
            elif isinstance(parsed_json, list): # si el modelo obedece y devuelve la lista directa
                room_sequence = parsed_json # usamos la lista tal cual

            self.get_logger().info(f"Sequence extracted by the LLM: {room_sequence}") # mostramos la secuencia extraida
            return room_sequence # devolvemos la lista de habitaciones

        except requests.exceptions.HTTPError as e: # si hay un error http del servidor de ollama
            self.get_logger().error(f"Ollama HTTP failure: {e.response.text}") # informamos del fallo http
            return [] # devolvemos una lista vacia
        except Exception as e: # si ocurre cualquier otro error con el modelo
            self.get_logger().error(f"General LLM error: {e}") # informamos del error general
            return [] # devolvemos una lista vacia

    def command_callback(self, msg): # funcion que se ejecuta cada vez que llega una orden al topic
        user_text = msg.data.strip() # leemos el texto del mensaje y quitamos espacios sobrantes

        if not user_text: # si el mensaje viene vacio
            self.get_logger().warn("Empty command received. Ignoring.") # avisamos y lo ignoramos
            return # salimos sin hacer nada

        self.get_logger().info(f"Command received from the user: '{user_text}'") # mostramos la orden recibida
        self.execute_semantic_command(user_text) # lanzamos el procesamiento completo de la orden

    def wants_to_discard(self, user_text): # funcion que decide si el usuario quiere descartar los destinos pendientes
        text_lower = user_text.lower() # pasamos la frase a minusculas para comparar sin importar mayusculas
        for keyword in self.discard_keywords: # recorremos todas las palabras clave de descarte
            if keyword in text_lower: # si alguna aparece en la frase del usuario
                return True # entonces el usuario quiere vaciar la cola anterior
        return False # si no aparece ninguna, el usuario quiere acumular el nuevo destino

    def execute_semantic_command(self, user_text): # tuberia principal: texto -> modelo -> coordenadas -> cola -> nav2
        # paso 1: obtener las habitaciones a partir del modelo
        target_rooms = self.extract_rooms_from_prompt(user_text) # pedimos al modelo la secuencia de habitaciones

        if not target_rooms: # si el modelo no ha devuelto ninguna habitacion valida
            self.get_logger().warn("No valid rooms found in the command.") # avisamos por terminal
            return # salimos sin tocar la cola

        # paso 2: convertir los nombres de habitaciones en coordenadas reales
        new_waypoints = [] # creamos la lista de nuevos puntos de paso
        for room_name in target_rooms: # recorremos cada habitacion que devolvio el modelo
            room_name_lower = room_name.lower() # pasamos el nombre a minusculas para comparar
            if room_name_lower in self.room_data: # si la habitacion existe en nuestro diccionario
                new_waypoints.append(self.room_data[room_name_lower]) # anadimos sus coordenadas a la lista
            else: # si el modelo se invento una habitacion que no existe
                self.get_logger().warn(f"The LLM suggested an unknown room: {room_name}") # avisamos y la descartamos

        if not new_waypoints: # si no hemos podido traducir ninguna habitacion a coordenadas
            self.get_logger().error("Could not translate any room into coordinates.") # informamos del error
            return # salimos sin tocar la cola

        # paso 3: decidir si descartamos lo anterior o acumulamos los nuevos destinos
        if self.wants_to_discard(user_text): # si la orden contiene una palabra de descarte (quedate, ya no, solo...)
            self.get_logger().warn("Orden de descarte detectada: vaciando la cola y cancelando el viaje actual.") # avisamos del descarte
            self.pending_goals = list(new_waypoints) # sustituimos toda la cola por los nuevos destinos
            # cancelamos el viaje en curso para que la silla cambie de rumbo inmediatamente
            if self.meta_actual is not None: # si la silla esta ahora mismo navegando hacia algun sitio
                self.meta_actual.cancel_goal_async() # pedimos a nav2 que cancele ese viaje
                self.meta_actual = None # olvidamos el manejador del viaje cancelado
                self.current_waypoint = None # descartamos tambien el destino interrumpido, no se vuelve a visitar
            self.send_next_goal() # lanzamos directamente el primer destino de la nueva cola
        else: # si no hay palabra de descarte, acumulamos en la cola (modo memoria)
            self.get_logger().info("Orden de acumular: anadiendo los nuevos destinos a la cola.") # avisamos que acumulamos
            # los nuevos destinos van al PRINCIPIO de la cola para visitarlos antes que lo que ya habia pendiente
            self.pending_goals = list(new_waypoints) + self.pending_goals # encolamos delante los destinos recien pedidos
            # si la silla estaba navegando, la desviamos ya hacia el nuevo destino; lo viejo queda guardado detras
            if self.meta_actual is not None: # si habia un viaje en curso
                self.get_logger().info("Desviando hacia el nuevo destino; el anterior queda guardado en la cola.") # explicamos el desvio
                self.meta_actual.cancel_goal_async() # cancelamos el viaje actual
                self.meta_actual = None # olvidamos el manejador del viaje cancelado
                self.pending_goals.append(self.current_waypoint) # el destino interrumpido vuelve a la cola, detras de los nuevos
                self.current_waypoint = None # ya no hay un destino "actual" hasta que se envie el siguiente
            self.send_next_goal() # arrancamos el primer destino de la cola (el recien anadido)

        self.get_logger().info(f"Cola de destinos pendientes: {len(self.pending_goals)}.") # mostramos cuantos destinos quedan

    def send_next_goal(self): # funcion que coge el primer destino de la cola y lo envia a nav2
        if self.meta_actual is not None: # si ya hay un viaje en curso no lanzamos otro encima
            return # esperamos a que termine el actual; al llegar se llamara de nuevo a esta funcion

        if not self.pending_goals: # si la cola esta vacia
            self.get_logger().info("Cola vacia: la silla ha llegado a su destino final.") # avisamos que no quedan destinos
            return # salimos porque no hay nada que enviar

        next_waypoint = self.pending_goals.pop(0) # sacamos el primer destino de la cola y lo quitamos de ella
        self.send_nav2_goal(next_waypoint) # enviamos ese destino a nav2

    def send_nav2_goal(self, waypoint): # funcion que construye el objetivo de nav2 y lo envia al servidor
        self.current_waypoint = waypoint # recordamos a que destino corresponde la meta que vamos a enviar
        self.get_logger().info("Esperando al servidor de accion 'NavigateThroughPoses'...") # avisamos que esperamos al servidor
        self.nav_client.wait_for_server() # bloqueamos hasta que el servidor de nav2 este disponible

        goal_msg = NavigateThroughPoses.Goal() # creamos el mensaje del objetivo vacio

        # empaquetamos el destino (x, y, orientacion) en un mensaje posestamped
        x, y, theta = waypoint # desempaquetamos las tres componentes del destino
        pose = PoseStamped() # creamos un mensaje de pose vacio
        pose.header.frame_id = 'map' # indicamos que la pose esta referida al mapa
        pose.header.stamp = self.get_clock().now().to_msg() # ponemos la marca de tiempo actual
        pose.pose.position.x = float(x) # fijamos la coordenada x del punto
        pose.pose.position.y = float(y) # fijamos la coordenada y del punto
        pose.pose.position.z = 0.0 # la altura es cero porque navegamos en 2d

        # magia matematica: convertir grados a cuaternion
        yaw_rad = math.radians(theta) # pasamos la orientacion de grados a radianes
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0) # calculamos la componente z del cuaternion
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0) # calculamos la componente w del cuaternion

        goal_msg.poses.append(pose) # anadimos la pose a la lista de puntos del objetivo

        # enviamos el objetivo de forma asincrona y pedimos que nos avisen cuando nav2 lo acepte o lo rechace
        self.get_logger().info(f"Enviando destino a Nav2. Quedan en cola: {len(self.pending_goals)}.") # avisamos del envio
        send_goal_future = self.nav_client.send_goal_async(goal_msg) # enviamos el objetivo a nav2
        send_goal_future.add_done_callback(self.goal_response_callback) # registramos el callback de respuesta

    def goal_response_callback(self, future): # callback que se ejecuta cuando nav2 acepta o rechaza el objetivo
        goal_handle = future.result() # obtenemos el manejador del objetivo enviado
        if not goal_handle.accepted: # si nav2 ha rechazado el objetivo
            self.get_logger().error('La meta fue rechazada por Nav2. Probando el siguiente destino de la cola.') # avisamos del rechazo
            self.send_next_goal() # intentamos con el siguiente destino de la cola
            return # salimos de este callback

        self.get_logger().info('Meta aceptada por Nav2. Robot en movimiento.') # avisamos que nav2 acepto el destino
        self.meta_actual = goal_handle # guardamos la meta actual para poder cancelarla si llega otra orden
        # pedimos que nos avisen cuando el viaje termine (llegada o fallo)
        result_future = goal_handle.get_result_async() # solicitamos el resultado de forma asincrona
        result_future.add_done_callback(self.reached_goal_callback) # registramos el callback de llegada

    def reached_goal_callback(self, future): # callback que se ejecuta cuando la silla termina un viaje
        self.get_logger().info("Destino alcanzado.") # avisamos que la silla ha llegado al destino
        self.meta_actual = None # marcamos que ya no hay viaje en curso
        self.current_waypoint = None # el destino que acabamos de alcanzar ya no es el "actual"
        self.send_next_goal() # lanzamos automaticamente el siguiente destino de la cola (si queda alguno)

def main(args=None): # funcion principal que arranca el nodo
    rclpy.init(args=args) # inicializamos las comunicaciones de ros 2
    node = SemanticNavigator() # creamos una instancia del navegador semantico

    try: # mantenemos el nodo vivo escuchando ordenes
        rclpy.spin(node) # cedemos el control a ros 2 para procesar los callbacks
    except KeyboardInterrupt: # si pulsamos ctrl+c en la terminal
        node.get_logger().info("Shutting down the Semantic Navigator...") # avisamos que vamos a apagar el nodo
    finally: # pase lo que pase liberamos los recursos
        node.destroy_node() # destruimos el nodo
        rclpy.shutdown() # cerramos las comunicaciones de ros 2

if __name__ == '__main__': # si ejecutamos este archivo directamente
    main() # llamamos a la funcion principal