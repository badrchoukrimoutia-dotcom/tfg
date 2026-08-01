import rclpy # importamos la libreria principal de ros 2 para python
from rclpy.node import Node # importamos la clase Node para crear nuestro propio nodo
import json # importamos json para procesar las respuestas del modelo de lenguaje
import requests # importamos requests para hacer peticiones http al servidor de ollama
from pathlib import Path # importamos Path para construir la ruta del json relativa a este archivo

from std_msgs.msg import String # importamos el mensaje de texto para recibir las ordenes del usuario y publicar el estado

from sap_executor import SapExecutor # importamos el ejecutor del sap: traduce cada accion a un objetivo de nav2
from semantic_grounding import ground # importamos el grounding: convierte las directivas del llm en un sap valido

class SemanticNavigator(Node): # definimos el nodo que traduce ordenes habladas en un sap y delega su ejecucion
    def __init__(self): # constructor donde preparamos todo el nodo
        super().__init__('semantic_navigator_node') # nombramos e inicializamos el nodo en la red de ros 2

        self.get_logger().info("Starting the Semantic Navigation Node...") # avisamos por terminal que el nodo arranca

        # --- estado de la memoria (version sencilla del mk = Mk, Gk, Pk, Hk del paper) ---
        # Gk (accion activa del sap) y Pk (cola de acciones pendientes del sap) ahora viven dentro del
        # sap_executor, que las gestiona como SemanticAction en vez de tuplas sueltas (ver sap_executor.py)
        self.interaction_history = [] # Hk: lista con el registro de lo que va pasando (para depurar y para el paper)
        self.active_constraints = [] # zonas a evitar activas ahora mismo (se acumulan/retiran orden a orden)
        self.active_preferences = [] # zonas con preferencia activa ahora mismo (se acumulan/retiran orden a orden)

        # retraso antes de enviar un objetivo a nav2 cuando la orden ha cambiado las restricciones/preferencias
        # activas, para darle tiempo a asimilar el mapa de costes actualizado (ver el comentario en
        # execute_semantic_command). parametro configurable, no numero fijo enterrado en la logica.
        self.costmap_sync_delay_seconds = 1.0
        self._pending_command_timer = None # temporizador de un solo disparo mientras esperamos ese retraso

        # palabras que, si aparecen en la orden, indican que el usuario quiere DESCARTAR lo que habia pendiente
        self.discard_keywords = ["quedate", "quédate", "solo", "solamente", "unicamente", "únicamente",
                                 "cancela", "olvida", "anula", "descarta", "ya no", "no vayas"] # disparadores del modo descartar

        # frases que, si aparecen en la orden, indican que el usuario quiere RETIRAR una restriccion o preferencia
        # (en vez de anadir una nueva). el llm no distingue esto de forma fiable (probado contra phi3), asi que
        # la deteccion se hace aqui, en el codigo, igual que discard_keywords para los destinos
        self.removal_keywords = ["deja de evitar", "ya puedes pasar por", "vuelve a permitir",
                                 "quita la restricción de", "quita la restriccion de",
                                 "ya no hace falta ir despacio por", "quita la preferencia de"]

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
        # publicador de las restricciones/preferencias activas, para poder visualizarlas durante las pruebas
        # y para que el nodo de inyeccion de costes (main.py) las escuche y actualice el mapa de costes
        self.constraints_publisher = self.create_publisher(String, '/active_constraints', 10) # topic nuevo en json
        # publicador de los eventos de feedback/replanteamiento (fase 3), para seguir el bucle durante las pruebas
        self.replanning_events_publisher = self.create_publisher(String, '/replanning_events', 10) # topic nuevo en json

        # el sap_executor ejecuta el sap: mantiene gk/pk sobre acciones semanticas, habla con nav2 por debajo,
        # y ante un fallo decide una politica de replanteamiento (reintentar o rendirse, ver sap_executor.py)
        self.sap_executor = SapExecutor(
            self, # el propio nodo, para que el executor pueda usar su logger, su reloj y sus temporizadores
            self.room_data, # diccionario de habitaciones para que el executor traduzca cada goto a coordenadas
            on_state_changed=self.publish_state, # cada vez que cambia gk/pk republicamos el estado cognitivo y el sap
            on_history_event=self.interaction_history.append, # cada evento de ejecucion (completado, fallido...) se registra en hk
            on_replanning_event=self.publish_replanning_event, # cada fallo + decision de replanteamiento se publica en json
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

    def extract_llm_directives(self, user_text): # funcion que pide al modelo destinos, restricciones y preferencias de la frase
        # le decimos al modelo exactamente que habitaciones existen para evitar alucinaciones
        available_rooms = list(self.room_data.keys()) # obtenemos la lista de nombres de habitaciones validas

        # el system prompt define las reglas estrictas de comportamiento del LLM (se mantiene en espanol).
        # ojo: aqui solo pedimos el caso de "anadir" (evitar una zona, ir despacio por una zona). el caso de
        # "retirar" una restriccion/preferencia (p. ej. "deja de evitar la cocina") NO se le pide al modelo:
        # probado contra phi3, no distingue de forma fiable si una frase asi es para anadir o quitar, asi que
        # esa deteccion se hace por separado en el codigo (ver removal_keywords / extract_removal_targets)
        system_prompt = (
            "Eres un analizador de datos estricto para un robot. Analiza la frase del usuario y clasifica la información en tres listas separadas. "
            f"LISTA ESTRICTA DE HABITACIONES PERMITIDAS: {available_rooms}. "
            "1. \"destinos\": habitaciones que el usuario quiere que el robot visite, en orden cronológico. "
            "REGLA DE CANCELACIÓN: si el usuario indica explícitamente que NO quiere ir a un sitio (ej: 'ya no vayas a', 'descarta', 'olvida'), NO incluyas esa habitación en destinos. "
            "2. \"restricciones\": habitaciones que el usuario quiere EVITAR o por las que NO quiere pasar el robot. "
            "3. \"preferencias\": habitaciones donde el usuario quiere que el robot vaya despacio o con velocidad reducida. "
            "REGLAS GENERALES: "
            "- NO traduzcas al inglés. Mantén los nombres exactamente como están en la lista en español. "
            "- IGNORA los objetos (gafas, llaves, etc.) que no sean habitaciones. "
            "- Si una habitación no está en la lista permitida, NO la incluyas en ninguna lista. "
            "- Una misma habitación puede aparecer en más de una lista si la frase lo justifica. "
            "- Devuelve SOLO un JSON con esta forma exacta: {\"destinos\": [], \"restricciones\": [], \"preferencias\": []}, usando arrays vacíos si no aplica."
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

        empty_directives = {"destinations": [], "constraints": [], "preferences": []} # valor por defecto si algo falla

        self.get_logger().info(f"Asking the LLM to process: '{user_text}'... (puede tardar si nav2/rviz estan usando la cpu)") # avisamos que estamos consultando al modelo

        try: # intentamos la comunicacion con el modelo de forma segura
            # 120 segundos de paciencia: en cpu (sin gpu) y con nav2+rviz compitiendo por los mismos
            # nucleos, una inferencia normal puede tardar bastante mas que los 60s originales
            response = requests.post(self.ollama_url, json=payload, timeout=120.0) # lanzamos la peticion al servidor local
            response.raise_for_status() # comprobamos que la respuesta http es correcta

            result_text = response.json().get("response", "{}") # extraemos el texto de la respuesta del modelo
            parsed_json = json.loads(result_text) # convertimos ese texto json en una estructura de python

            directives = { # rescatamos cada categoria buscando una clave que encaje, venga como venga del modelo
                "destinations": self._extract_directive_list(parsed_json, ("destin",)),
                "constraints": self._extract_directive_list(parsed_json, ("restric", "evit")),
                "preferences": self._extract_directive_list(parsed_json, ("prefer", "despacio", "lent")),
            }

            self.get_logger().info(f"LLM directives -> destinos: {directives['destinations']}, restricciones: {directives['constraints']}, preferencias: {directives['preferences']}") # mostramos lo extraido
            return directives

        except requests.exceptions.HTTPError as e: # si hay un error http del servidor de ollama
            self.get_logger().error(f"Ollama HTTP failure: {e.response.text}") # informamos del fallo http
            return dict(empty_directives) # devolvemos las tres listas vacias
        except Exception as e: # si ocurre cualquier otro error con el modelo
            self.get_logger().error(f"General LLM error: {e}") # informamos del error general
            return dict(empty_directives) # devolvemos las tres listas vacias

    def _extract_directive_list(self, parsed_json, key_hints): # busca en el json del llm la lista de esta categoria, aunque el modelo use otra clave
        if not isinstance(parsed_json, dict): # si el modelo no devolvio un diccionario no hay categorias que buscar
            return []
        for key, value in parsed_json.items(): # recorremos las claves del diccionario que devolvio el modelo
            key_lower = key.lower() # comparamos en minusculas para no depender de mayusculas
            if any(hint in key_lower for hint in key_hints): # si la clave se parece a la categoria que buscamos (p. ej. "preferences" tambien vale para "prefer")
                if isinstance(value, list): # caso normal: el valor ya es una lista
                    return value
                if isinstance(value, str) and value: # el modelo a veces devuelve un string suelto en vez de una lista de un elemento
                    return [value]
        return [] # no encontramos ninguna clave que encaje con esta categoria

    def wants_to_remove_constraints(self, user_text): # funcion que decide si el usuario quiere RETIRAR restricciones/preferencias (en vez de anadir)
        text_lower = user_text.lower() # pasamos la frase a minusculas para comparar sin importar mayusculas
        for keyword in self.removal_keywords: # recorremos todas las palabras clave de eliminacion
            if keyword in text_lower: # si alguna aparece en la frase del usuario
                return True # entonces el usuario quiere retirar, no anadir
        return False # si no aparece ninguna, es una orden normal de anadir

    def extract_removal_targets(self, user_text): # busca en el texto, sin pasar por el llm, que habitaciones conocidas se mencionan
        text_lower = user_text.lower() # pasamos la frase a minusculas para comparar sin importar mayusculas
        matches = [room for room in self.room_data.keys() if room in text_lower] # habitaciones cuyo nombre aparece en la frase
        # evitamos falsos positivos por solapamiento de nombres (p. ej. "pasillo" dentro de "continuación del pasillo"):
        # si una coincidencia es a su vez substring de otra coincidencia mas larga, nos quedamos solo con la mas larga
        return [room for room in matches if not any(room != other and room in other for other in matches)]

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

    def execute_semantic_command(self, user_text): # tuberia principal: texto -> modelo -> grounding -> memoria -> nav2/costes
        # paso 1: obtener destinos, restricciones y preferencias a partir del modelo
        directives = self.extract_llm_directives(user_text) # pedimos al modelo las tres listas

        # paso 2: grounding semantico: convertimos las directivas del llm en un sap valido (nombres ya comprobados contra el mapa)
        plan, unknown_names = ground(directives, self.room_data)
        for category, names in unknown_names.items(): # avisamos de cualquier habitacion que el llm se haya inventado, por categoria
            for name in names:
                self.get_logger().warn(f"The LLM suggested an unknown room ({category}): {name}")

        # paso 3: restricciones y preferencias: segun la orden, se FUSIONAN con las activas o se RETIRAN de ellas
        removal_mode = self.wants_to_remove_constraints(user_text) # marcador de eliminacion detectado (ver removal_keywords)
        constraints_changed = removal_mode or bool(plan.constraints or plan.preferences) # si esta orden toca el conjunto activo

        if removal_mode:
            removal_targets = self.extract_removal_targets(user_text) # zonas mencionadas, buscadas directamente en el texto (sin llm)
            self._remove_constraints_and_preferences(removal_targets)
            self.get_logger().info(f"Restricciones/preferencias retiradas -> {removal_targets}") # avisamos de la retirada
            self.interaction_history.append(f"retirar restriccion/preferencia -> {removal_targets}") # Hk: registramos la orden
        elif plan.constraints or plan.preferences: # orden normal: se anaden/actualizan restricciones o preferencias
            self._merge_constraints_and_preferences(plan)
            self.get_logger().info(
                f"Restricciones activas: {[c.target for c in self.active_constraints]}. "
                f"Preferencias activas: {[p.target for p in self.active_preferences]}."
            ) # mostramos el conjunto activo tras la fusion
            self.interaction_history.append(
                f"restriccion/preferencia -> {[c.target for c in plan.constraints]} / {[p.target for p in plan.preferences]}"
            ) # Hk: registramos la orden en el historial

        if constraints_changed:
            self.publish_active_constraints() # republicamos en cuanto cambie el conjunto activo de restricciones/preferencias

        # paso 4: destinos (sin cambios respecto a la fase 1, ahora usando las acciones ya construidas por el grounding)
        if not plan.actions: # si el modelo no ha devuelto ninguna habitacion destino valida
            self.get_logger().warn("No valid rooms found in the command.") # avisamos por terminal
            return # salimos sin tocar la memoria de destinos (las restricciones/preferencias ya se han procesado arriba)

        destination_names = [action.target for action in plan.actions] # nombres de los destinos, para el log y el historial

        # paso 5: decidir si DESCARTAMOS lo anterior o si INTERRUMPIMOS para retomar despues (acumular)
        discard = self.wants_to_discard(user_text) # si la orden contiene una palabra de descarte (quedate, ya no, solo...)
        if discard:
            self.get_logger().warn("Orden de DESCARTE detectada: se vacia la cola y no se retoma nada.") # avisamos del descarte
            self.interaction_history.append(f"descartar -> {destination_names}") # Hk: registramos la orden en el historial
        else: # si NO hay palabra de descarte: interrumpimos el actual, lo guardamos y lo retomamos despues (lo que pide el paper)
            self.get_logger().info("Orden de ACUMULAR: el nuevo destino se hace ahora y lo anterior se retomara despues.") # avisamos
            self.interaction_history.append(f"acumular -> {destination_names}") # Hk: registramos la orden en el historial

        # paso 6: ejecucion del sap: el sap_executor decide gk/pk, interrumpe si toca, y habla con nav2 por debajo.
        # si esta orden ha cambiado las restricciones/preferencias activas, esperamos un poco antes de enviar el
        # objetivo: el costmap global de nav2 solo se refresca cada 1/update_frequency segundos (ver
        # global_costmap.update_frequency en mi_configuracion.yaml), asi que si enviamos el goal justo despues de
        # publicar el mapa de costes actualizado, nav2 puede planificar todavia con el mapa viejo y atravesar una
        # zona que deberiamos estar evitando. este retraso fijo es una solucion suficiente para el simulador; una
        # solucion mas robusta seria esperar una confirmacion explicita de que el costmap ya se ha refrescado
        # (p. ej. consultando el servicio de costmap de nav2), pero para el alcance de este trabajo basta con esto.
        if constraints_changed:
            self.get_logger().info(
                f"Esperando {self.costmap_sync_delay_seconds}s a que Nav2 asimile el mapa de costes actualizado..."
            ) # avisamos de la espera

            def send_after_costmap_sync(): # se ejecuta una unica vez, cuando pasa el retraso configurado
                self._pending_command_timer.cancel() # temporizador de un solo disparo: se cancela nada mas saltar
                self._pending_command_timer = None
                self.sap_executor.enqueue(plan.actions, discard)
                pending_names = [action.target for action in self.sap_executor.pending_actions] # nombres de lo pendiente, para el log
                self.get_logger().info(f"Cola de acciones pendientes (Pk): {pending_names}.") # mostramos el contenido actual de la cola

            self._pending_command_timer = self.create_timer(self.costmap_sync_delay_seconds, send_after_costmap_sync)
            return

        self.sap_executor.enqueue(plan.actions, discard)

        pending_names = [action.target for action in self.sap_executor.pending_actions] # nombres de lo que queda pendiente, para el log
        self.get_logger().info(f"Cola de acciones pendientes (Pk): {pending_names}.") # mostramos el contenido actual de la cola

    def _merge_constraints_and_preferences(self, plan): # fusiona las restricciones/preferencias nuevas con las activas (por tipo+zona, sin duplicar)
        for constraint in plan.constraints: # recorremos cada restriccion nueva del plan
            self.active_constraints = [ # quitamos cualquier restriccion previa del mismo tipo sobre la misma zona
                c for c in self.active_constraints
                if not (c.constraint_type == constraint.constraint_type and c.target == constraint.target)
            ]
            self.active_constraints.append(constraint) # anadimos la version nueva (fusion = sustituir + anadir)
        for preference in plan.preferences: # recorremos cada preferencia nueva del plan
            self.active_preferences = [ # quitamos cualquier preferencia previa del mismo tipo sobre la misma zona
                p for p in self.active_preferences
                if not (p.preference_type == preference.preference_type and p.target == preference.target)
            ]
            self.active_preferences.append(preference) # anadimos la version nueva

    def _remove_constraints_and_preferences(self, targets): # retira de las activas cualquier restriccion/preferencia sobre alguna de estas zonas
        self.active_constraints = [c for c in self.active_constraints if c.target not in targets]
        self.active_preferences = [p for p in self.active_preferences if p.target not in targets]

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

    def publish_active_constraints(self): # publica las restricciones/preferencias activas ahora mismo, para observarlas y para main.py
        summary = { # construimos un resumen legible del conjunto activo
            "restricciones": [c.to_dict() for c in self.active_constraints],
            "preferencias": [p.to_dict() for p in self.active_preferences],
        }
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(summary, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.constraints_publisher.publish(msg) # publicamos en el topic /active_constraints

    def publish_semantic_action_plan(self): # publica el sap completo (accion activa + pendientes, con su estado) para observarlo en las pruebas
        plan = [action.to_dict() for action in self.sap_executor.full_plan] # sap completo en orden: gk primero, luego pk
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(plan, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.sap_publisher.publish(msg) # publicamos el sap en el topic /semantic_action_plan

    def publish_replanning_event(self, event): # publica un evento de feedback/replanteamiento (fallo + decision tomada) en json
        msg = String() # creamos el mensaje de texto que vamos a publicar
        msg.data = json.dumps(event, ensure_ascii=False, indent=2) # lo pasamos a json legible conservando las tildes
        self.replanning_events_publisher.publish(msg) # publicamos en el topic /replanning_events

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
