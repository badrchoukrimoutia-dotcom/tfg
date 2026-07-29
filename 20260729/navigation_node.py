import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
import json # importamos json para procesar las respuestas del modelo de lenguaje
import requests # importamos requests para hacer peticiones http al servidor de ollama
from pathlib import Path # importamos Path para construir la ruta del json relativa a este archivo

from std_msgs.msg import String # importamos el mensaje de texto para recibir las ordenes del usuario y publicar el estado

from semantic_action_plan import build_sap # importamos la generacion del sap: nombres de habitacion -> acciones goto
from sap_executor import SapExecutor # importamos el ejecutor del sap: traduce cada accion a un objetivo de nav2

class SemanticNavigator(Node): # definimos el nodo que traduce ordenes habladas en un sap y delega su ejecucion
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('semantic_navigator_node') # nombramos e inicializamos el nodo en la red de ros 2

        self.get_logger().info("Starting the Semantic Navigation Node...") # avisamos por terminal que el nodo arranca

        # --- estado de la memoria (version sencilla del mk = Mk, Gk, Pk, Hk del paper) ---
        # Gk (accion activa del sap) y Pk (cola de acciones pendientes del sap) ahora viven dentro del
        # sap_executor, que las gestiona como SemanticAction en vez de tuplas sueltas (ver sap_executor.py)
        self.interaction_history = [] # Hk: lista con el registro de lo que va pasando (para depurar y para el paper)

        # palabras que, si aparecen en la orden, indican que el usuario quiere DESCARTAR lo que habia pendiente
        self.discard_keywords = ["quedate", "quédate", "solo", "solamente", "unicamente", "únicamente",
                                 "cancela", "olvida", "anula", "descarta", "ya no", "no vayas"] # disparadores del modo descartar

        # cargamos el mapa arquitectonico desde el json usando una ruta relativa a este archivo (asi no depende de la carpeta)
        self.map_filepath = Path(__file__).resolve().parent / 'ejemplo_natalia.json' # ruta del json del entorno junto al script
        self.room_data = self.load_map_data(self.map_filepath) # construimos el diccionario de habitaciones y coordenadas

        # configuramos el punto de acceso de la api de ollama que corre en local
        self.ollama_url = "http://localhost:11434/api/generate" # direccion del servidor local del modelo de lenguaje
        self.llm_model = "phi3" # modelo que usamos, se puede cambiar por llama3 segun lo instalado en la zimaboard

        # publicador del estado de la memoria (Gk, Pk, Hk) para poder visualizarlo durante los experimentos del paper
        self.cognitive_state_publisher = self.create_publisher(String, '/cognitive_state', 10) # topic con el estado en json
        # publicador del sap actual (accion activa + pendientes, con su estado), para poder visualizarlo durante las pruebas
        self.sap_publisher = self.create_publisher(String, '/semantic_action_plan', 10) # topic nuevo con el sap en json

        # el sap_executor ejecuta el sap: mantiene gk/pk sobre acciones semanticas y habla con nav2 por debajo
        self.sap_executor = SapExecutor(
            self, # el propio nodo, para que el executor pueda usar su logger y su reloj
            self.room_data, # diccionario de habitaciones para que el executor traduzca cada goto a coordenadas
            on_state_changed=self.publish_state, # cada vez que cambia gk/pk republicamos el estado cognitivo y el sap
            on_history_event=self.interaction_history.append, # cada evento de ejecucion (completado, fallido...) se registra en hk
        )

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

        # paso 2: filtrar solo las habitaciones que existen de verdad en el mapa (evita alucinaciones del LLM)
        valid_room_names = [] # nombres de habitacion validos, con los que construiremos el sap
        for room_name in target_rooms: # recorremos cada habitacion que devolvio el modelo
            room_name_lower = room_name.lower() # pasamos el nombre a minusculas para comparar
            if room_name_lower in self.room_data: # si la habitacion existe en nuestro diccionario
                valid_room_names.append(room_name_lower) # la anadimos a la lista de nombres validos
            else: # si el modelo se invento una habitacion que no existe
                self.get_logger().warn(f"The LLM suggested an unknown room: {room_name}") # avisamos y la descartamos

        if not valid_room_names: # si no hemos podido validar ninguna habitacion
            self.get_logger().error("Could not translate any room into a valid sap action.") # informamos del error
            return # salimos sin tocar la memoria

        # paso 3: generacion del sap: construimos la lista ordenada de acciones goto (etapa separada de su ejecucion)
        sap = build_sap(valid_room_names)

        # paso 4: decidir si DESCARTAMOS lo anterior o si INTERRUMPIMOS para retomar despues (acumular)
        discard = self.wants_to_discard(user_text) # si la orden contiene una palabra de descarte (quedate, ya no, solo...)
        if discard:
            self.get_logger().warn("Orden de DESCARTE detectada: se vacia la cola y no se retoma nada.") # avisamos del descarte
            self.interaction_history.append(f"descartar -> {valid_room_names}") # Hk: registramos la orden en el historial
        else: # si NO hay palabra de descarte: interrumpimos el actual, lo guardamos y lo retomamos despues (lo que pide el paper)
            self.get_logger().info("Orden de ACUMULAR: el nuevo destino se hace ahora y lo anterior se retomara despues.") # avisamos
            self.interaction_history.append(f"acumular -> {valid_room_names}") # Hk: registramos la orden en el historial

        # paso 5: ejecucion del sap: el sap_executor decide gk/pk, interrumpe si toca, y habla con nav2 por debajo
        self.sap_executor.enqueue(sap, discard)

        pending_names = [action.target for action in self.sap_executor.pending_actions] # nombres de lo que queda pendiente, para el log
        self.get_logger().info(f"Cola de acciones pendientes (Pk): {pending_names}.") # mostramos el contenido actual de la cola

    def publish_state(self): # republica tanto el estado cognitivo (Gk/Pk/Hk) como el sap, cada vez que algo cambia
        self.publish_cognitive_state()
        self.publish_semantic_action_plan()

    def publish_cognitive_state(self): # publica el estado de la memoria (Gk, Pk, Hk) para depurar y visualizar los experimentos
        current_action = self.sap_executor.current_action # gk: la accion del sap en ejecucion, o none si esta parada
        summary = { # construimos un resumen legible del estado actual
            "objetivo_activo_Gk": current_action.target if current_action is not None else None, # a donde va ahora
            "pendientes_Pk": [action.target for action in self.sap_executor.pending_actions], # que le queda por visitar, en orden
            "historial_Hk": self.interaction_history[-10:], # las ultimas 10 cosas que han pasado, para no saturar
        }
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(summary, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.cognitive_state_publisher.publish(msg) # publicamos el estado en el topic /cognitive_state

    def publish_semantic_action_plan(self): # publica el sap completo (accion activa + pendientes, con su estado) para observarlo en las pruebas
        plan = [action.to_dict() for action in self.sap_executor.full_plan] # sap completo en orden: gk primero, luego pk
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(plan, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.sap_publisher.publish(msg) # publicamos el sap en el topic /semantic_action_plan

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
