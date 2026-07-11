import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
from rclpy.action import ActionClient # importamos el cliente de acciones para comunicarnos con nav2
import json # importamos json para procesar las respuestas del modelo de lenguaje
import requests # importamos requests para hacer peticiones http al servidor de ollama
import math # importamos math para convertir angulos en cuaterniones
from pathlib import Path # importamos Path para construir la ruta del json relativa a este archivo

from nav2_msgs.action import NavigateThroughPoses # importamos la accion de nav2 para navegar por varios puntos seguidos
from action_msgs.msg import GoalStatus # importamos los codigos de estado para saber si un objetivo termino con exito, cancelado o fallido
from geometry_msgs.msg import PoseStamped # importamos el mensaje de pose que usa ros 2 para las coordenadas
from std_msgs.msg import String # importamos el mensaje de texto para recibir las ordenes del usuario y publicar el estado

class SemanticNavigator(Node): # definimos el nodo que traduce ordenes habladas en navegacion
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('semantic_navigator_node') # nombramos e inicializamos el nodo en la red de ros 2

        self.get_logger().info("Starting the Semantic Navigation Node...") # avisamos por terminal que el nodo arranca

        # --- estado de la memoria (version sencilla del mk = Mk, Gk, Pk, Hk del paper) ---
        self.active_goal = None # Gk: el destino que la silla persigue AHORA (nombre + coordenadas), o None si esta parada
        self.pending_tasks = [] # Pk: cola de destinos que quedan por visitar (cada uno es una tupla nombre, x, y, theta)
        self.interaction_history = [] # Hk: lista con el registro de lo que va pasando (para depurar y para el paper)
        self.meta_actual = None # manejador ros 2 del objetivo en curso en nav2; nos permite cancelarlo (es distinto de active_goal)
        # True desde que send_next_goal() envia un objetivo a nav2 hasta que goal_response_callback confirma
        # si fue aceptado o rechazado. Sin esta bandera, si llega una orden nueva justo en ese hueco (active_goal
        # ya puesto pero meta_actual todavia None porque nav2 no ha contestado), send_next_goal() no detecta que
        # hay un envio en curso y manda un SEGUNDO goal encima, perdiendo la referencia al primero sin guardarlo
        # en pending_tasks: la tarea interrumpida deja de poder retomarse, justo lo que el paper pide evitar.
        self.goal_in_flight = False

        # palabras que, si aparecen en la orden, indican que el usuario quiere DESCARTAR lo que habia pendiente
        self.discard_keywords = ["quedate", "quédate", "solo", "solamente", "unicamente", "únicamente",
                                 "cancela", "olvida", "anula", "descarta", "ya no", "no vayas"] # disparadores del modo descartar

        # cargamos el mapa arquitectonico desde el json usando una ruta relativa a este archivo (asi no depende de la carpeta)
        self.map_filepath = Path(__file__).resolve().parent / 'ejemplo_natalia.json' # ruta del json del entorno junto al script
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

        # publicador del estado de la memoria (Gk, Pk, Hk) para poder visualizarlo durante los experimentos del paper
        self.cognitive_state_publisher = self.create_publisher(String, '/cognitive_state', 10) # topic nuevo con el estado en json

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

        # el system prompt define las reglas estrictas de comportamiento del LLM (se mantiene en espanol)
        system_prompt = (
            "Eres un analizador de datos estricto para un robot. "
            "Extrae de la frase del usuario ÚNICAMENTE los nombres de las habitaciones que debe visitar, en orden cronológico. "
            f"LISTA ESTRICTA DE HABITACIONES PERMITIDAS: {available_rooms}. "
            "REGLAS OBLIGATORIAS: "
            "1. NO traduzcas al inglés. Mantén los nombres exactamente como están en la lista en español. "
            "2. IGNORA los objetos (gafas, llaves, etc.) y las acciones. "
            "3. REGLA DE CANCELACIÓN: Si el usuario indica explícitamente que NO quiere ir a un sitio (ej: 'ya no vayas a', 'descarta', 'olvida'), IGNORA ESA HABITACIÓN por completo. "
            "4. Si una habitación no está en la lista, NO la incluyas. "
            "5. Devuelve SOLO un array JSON de strings con las habitaciones válidas."
        )

        payload = { # preparamos los datos que enviaremos al modelo
            "model": self.llm_model, # indicamos que modelo debe usar
            "prompt": f"Comando del usuario: '{user_text}'\n\nSalida esperada:", # le pasamos la frase del usuario
            "system": system_prompt, # adjuntamos las reglas del sistema
            "stream": False, # pedimos la respuesta completa de golpe, no en trozos
            "format": "json", # obligamos a ollama a devolver un json valido
            # sin esto, ollama descarga el modelo de memoria tras 5 min de inactividad (valor por defecto
            # del servidor) y la SIGUIENTE orden paga el coste entero de recargar ~3-4 gb de pesos antes
            # de poder generar nada. Con esta silla sin gpu (ollama corriendo 100% en cpu, ver 'ollama ps'),
            # esa recarga + la cpu que ya se esta usando en nav2/rviz es justo lo que provoca el timeout.
            "keep_alive": "30m" # mantenemos el modelo cargado 30 minutos entre ordenes en vez de 5
        }

        self.get_logger().info(f"Asking the LLM to process: '{user_text}'... (puede tardar si nav2/rviz estan usando la cpu)") # avisamos que estamos consultando al modelo

        try: # intentamos la comunicacion con el modelo de forma segura
            # 120 segundos de paciencia: en cpu (sin gpu) y con nav2+rviz compitiendo por los mismos
            # nucleos, una inferencia normal puede tardar bastante mas que los 60s originales
            response = requests.post(self.ollama_url, json=payload, timeout=120.0) # lanzamos la peticion al servidor local
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
        return False # si no aparece ninguna, el usuario quiere acumular (interrumpir y retomar despues)

    def execute_semantic_command(self, user_text): # tuberia principal: texto -> modelo -> coordenadas -> memoria -> nav2
        # paso 1: obtener las habitaciones a partir del modelo
        target_rooms = self.extract_rooms_from_prompt(user_text) # pedimos al modelo la secuencia de habitaciones

        if not target_rooms: # si el modelo no ha devuelto ninguna habitacion valida
            self.get_logger().warn("No valid rooms found in the command.") # avisamos por terminal
            return # salimos sin tocar la memoria

        # paso 2: convertir los nombres de habitaciones en waypoints (nombre + coordenadas, para poder loguearlos)
        new_waypoints = [] # creamos la lista de nuevos puntos de paso
        for room_name in target_rooms: # recorremos cada habitacion que devolvio el modelo
            room_name_lower = room_name.lower() # pasamos el nombre a minusculas para comparar
            if room_name_lower in self.room_data: # si la habitacion existe en nuestro diccionario
                x, y, theta = self.room_data[room_name_lower] # desempaquetamos las coordenadas guardadas para esa habitacion
                new_waypoints.append((room_name_lower, x, y, theta)) # guardamos nombre + coordenadas juntos, para el historial y los logs
            else: # si el modelo se invento una habitacion que no existe
                self.get_logger().warn(f"The LLM suggested an unknown room: {room_name}") # avisamos y la descartamos

        if not new_waypoints: # si no hemos podido traducir ninguna habitacion a coordenadas
            self.get_logger().error("Could not translate any room into coordinates.") # informamos del error
            return # salimos sin tocar la memoria

        new_names = [w[0] for w in new_waypoints] # lista de solo los nombres, util para el historial y los logs

        # paso 3: decidir si DESCARTAMOS lo anterior o si INTERRUMPIMOS para retomar despues (acumular)
        if self.wants_to_discard(user_text): # si la orden contiene una palabra de descarte (quedate, ya no, solo...)
            self.get_logger().warn("Orden de DESCARTE detectada: se vacia la cola y no se retoma nada.") # avisamos del descarte
            self.interaction_history.append(f"descartar -> {new_names}") # Hk: registramos la orden en el historial
            self.pending_tasks = list(new_waypoints) # Pk: la cola se sustituye por completo por los nuevos destinos
            if self.active_goal is not None: # si la silla tenia algun objetivo en curso (aceptado por nav2 o todavia esperando respuesta)
                self.meta_actual = None # olvidamos el manejador; nav2 sustituira el objetivo solo al llegar el nuevo (preemption)
                self.goal_in_flight = False # el envio de mas abajo sera el unico goal en vuelo a partir de ahora
                self.active_goal = None # Gk: el destino interrumpido se descarta, NO se guarda para retomarlo
                self.get_logger().info("Objetivo activo descartado (no se retomara).") # log claro para los experimentos
            self.send_next_goal() # lanzamos directamente el primer destino de la nueva cola
        else: # si NO hay palabra de descarte: interrumpimos el actual, lo guardamos y lo retomamos despues (lo que pide el paper)
            self.get_logger().info("Orden de ACUMULAR: el nuevo destino se hace ahora y lo anterior se retomara despues.") # avisamos
            self.interaction_history.append(f"acumular -> {new_names}") # Hk: registramos la orden en el historial
            # los nuevos destinos van al PRINCIPIO de la cola, para visitarlos ANTES que lo que ya habia pendiente
            self.pending_tasks = list(new_waypoints) + self.pending_tasks # Pk: encolamos delante los destinos recien pedidos
            # si la silla estaba navegando (aceptado por nav2 o todavia esperando respuesta), guardamos el
            # destino interrumpido para retomarlo cuando acaben los nuevos. Miramos active_goal, no meta_actual:
            # active_goal se pone ANTES de que nav2 confirme la aceptacion, asi que es la unica forma fiable de
            # detectar "hay un objetivo en curso" durante ese hueco (ver comentario de goal_in_flight en __init__).
            if self.active_goal is not None: # si efectivamente habia un destino activo que interrumpir
                self.meta_actual = None # olvidamos el manejador; nav2 sustituira el objetivo solo al llegar el nuevo (preemption)
                self.goal_in_flight = False # el envio de mas abajo sera el unico goal en vuelo a partir de ahora
                # IMPORTANTE: lo ponemos al FINAL de la cola, para que se retome despues de TODO lo nuevo
                self.pending_tasks.append(self.active_goal) # Gk -> Pk: el destino interrumpido pasa a pendientes (se retomara)
                self.get_logger().info(f"Objetivo interrumpido y guardado para retomar: {self.active_goal[0]}.") # log claro
                self.active_goal = None # Gk: dejamos vacio el objetivo activo hasta que send_next_goal ponga el nuevo
            self.send_next_goal() # arrancamos el primer destino de la cola (el recien anadido)

        pending_names = [w[0] for w in self.pending_tasks] # nombres de lo que queda pendiente, para el log
        self.get_logger().info(f"Cola de destinos pendientes (Pk): {pending_names}.") # mostramos el contenido actual de la cola
        self.publish_cognitive_state() # publicamos el nuevo estado de la memoria tras procesar la orden

    def send_next_goal(self): # funcion que coge el primer pendiente de la cola y lo envia a nav2
        if self.meta_actual is not None or self.goal_in_flight: # si ya hay un viaje en curso (aceptado o esperando respuesta) no lanzamos otro encima
            return # esperamos a que termine/se confirme el actual; al llegar se llamara de nuevo a esta funcion

        if not self.pending_tasks: # si la cola de pendientes esta vacia
            self.get_logger().info("Cola vacia: la silla ha llegado a su destino final.") # avisamos que no quedan destinos
            self.active_goal = None # Gk: no hay objetivo activo
            return # salimos porque no hay nada que enviar

        next_waypoint = self.pending_tasks.pop(0) # Pk: sacamos el primer pendiente de la cola (y lo quitamos de ella)
        self.active_goal = next_waypoint # Pk -> Gk: ese destino pasa a ser el objetivo activo
        self.goal_in_flight = True # marcamos que ya hay un envio en curso ANTES de que nav2 confirme la aceptacion
        self.get_logger().info(f"Objetivo activo (Gk): {next_waypoint[0]}.") # log claro para los experimentos
        self.send_nav2_goal(next_waypoint) # enviamos ese destino a nav2

    def send_nav2_goal(self, waypoint): # funcion que construye el objetivo de nav2 y lo envia al servidor
        self.get_logger().info("Esperando al servidor de acción 'NavigateThroughPoses'...") # avisamos que esperamos al servidor
        self.nav_client.wait_for_server() # bloqueamos hasta que el servidor de nav2 este disponible

        goal_msg = NavigateThroughPoses.Goal() # creamos el mensaje del objetivo vacio

        # desempaquetamos el waypoint (nombre, x, y, orientacion) y lo metemos en un mensaje posestamped
        name, x, y, theta = waypoint # separamos las cuatro partes del destino
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
        self.get_logger().info(f"Enviando destino a Nav2: {name}. Quedan en cola (Pk): {len(self.pending_tasks)}.") # avisamos del envio
        send_goal_future = self.nav_client.send_goal_async(goal_msg) # enviamos el objetivo a nav2
        send_goal_future.add_done_callback(self.goal_response_callback) # registramos el callback de respuesta

    def goal_response_callback(self, future): # callback que se ejecuta cuando nav2 acepta o rechaza el objetivo
        self.goal_in_flight = False # la respuesta ya llego: deja de estar "en vuelo" (aceptado o rechazado, cualquiera de los dos)
        goal_handle = future.result() # obtenemos el manejador del objetivo enviado
        if not goal_handle.accepted: # si nav2 ha rechazado el objetivo
            rejected_name = self.active_goal[0] if self.active_goal is not None else "desconocido" # nombre para el log
            self.get_logger().error(f"La meta fue rechazada por Nav2: {rejected_name}. Probando el siguiente de la cola.") # avisamos del rechazo
            self.interaction_history.append(f"rechazado -> {rejected_name}") # Hk: registramos el rechazo tambien en el historial
            self.active_goal = None # Gk: el rechazado se descarta como objetivo activo
            self.send_next_goal() # intentamos con el siguiente destino de la cola
            self.publish_cognitive_state() # publicamos el estado tras el rechazo
            return # salimos de este callback

        self.get_logger().info('Meta aceptada por Nav2. Robot en movimiento.') # avisamos que nav2 acepto el destino
        self.meta_actual = goal_handle # guardamos la meta actual para poder cancelarla si llega otra orden
        # pedimos que nos avisen cuando el viaje termine, atando el callback a ESTE goal_handle concreto.
        # asi, si el objetivo es sustituido por una orden mas nueva, el resultado tardio del viejo no se
        # confunde con el nuevo ni hace avanzar la cola dos veces.
        result_future = goal_handle.get_result_async() # solicitamos el resultado de forma asincrona
        result_future.add_done_callback(
            lambda f, gh=goal_handle: self.reached_goal_callback(f, gh)
        ) # registramos el callback de llegada, ligado a este objetivo concreto

    def reached_goal_callback(self, future, goal_handle): # callback que se ejecuta cuando el RESULTADO de un objetivo esta disponible
        if goal_handle is not self.meta_actual: # si este resultado ya no corresponde al objetivo que seguimos persiguiendo
            # el objetivo fue sustituido por una orden mas reciente; ignoramos su resultado tardio para no avanzar la cola dos veces
            self.get_logger().info("Resultado tardio de un objetivo ya sustituido; se ignora.") # log para que quede claro
            return # no tocamos la memoria ni la cola

        status = future.result().status # estado final del objetivo: succeeded, canceled o aborted
        reached_name = self.active_goal[0] if self.active_goal is not None else "desconocido" # nombre para el log y el historial

        if status == GoalStatus.STATUS_SUCCEEDED: # la silla llego de verdad al destino
            self.get_logger().info(f"Destino alcanzado: {reached_name}.") # avisamos que la silla ha llegado
            self.interaction_history.append(f"completado -> {reached_name}") # Hk: registramos la llegada en el historial
        else: # cualquier otro estado (p.ej. aborted): nav2 no pudo completar el objetivo
            self.get_logger().warn(f"El objetivo a '{reached_name}' termino sin exito (status={status}); seguimos con la cola.") # avisamos del fallo
            self.interaction_history.append(f"fallido -> {reached_name}") # Hk: registramos el fallo

        self.active_goal = None # Gk: el destino ya no es el objetivo activo (llegamos o fallo)
        self.meta_actual = None # marcamos que ya no hay viaje en curso
        self.send_next_goal() # lanzamos el siguiente de la cola; AQUI es donde se RETOMA lo que estaba interrumpido (Pk -> Gk)
        self.publish_cognitive_state() # publicamos el estado tras completar y, si tocaba, retomar el pendiente

    def publish_cognitive_state(self): # publica el estado de la memoria (Gk, Pk, Hk) para depurar y visualizar los experimentos
        summary = { # construimos un resumen legible del estado actual
            "objetivo_activo_Gk": self.active_goal[0] if self.active_goal is not None else None, # a donde va ahora
            "pendientes_Pk": [w[0] for w in self.pending_tasks], # que le queda por visitar, en orden
            "historial_Hk": self.interaction_history[-10:], # las ultimas 10 cosas que han pasado, para no saturar
        }
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(summary, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.cognitive_state_publisher.publish(msg) # publicamos el estado en el topic /cognitive_state

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
