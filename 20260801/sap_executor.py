import math # importamos math para convertir angulos en cuaterniones

from rclpy.action import ActionClient # cliente de accion para comunicarnos con nav2
from nav2_msgs.action import NavigateThroughPoses # accion de nav2 para navegar por varios puntos seguidos
from action_msgs.msg import GoalStatus # codigos de estado para saber si un objetivo termino con exito, cancelado o fallido
from geometry_msgs.msg import PoseStamped # mensaje de pose que usa ros 2 para las coordenadas

from semantic_action_plan import ActionStatus # estados posibles de una accion del sap
from execution_feedback import ExecutionFeedbackManager, FailureReason # tarea 1: registra el resultado de cada accion
from replanning import ReplanningPolicy, ReplanningDecision # tarea 2: decide que politica aplicar ante un fallo


class SapExecutor: # ejecuta el sap: traduce cada accion goto en un objetivo de nav2, sigue su resultado y replantea si falla
    def __init__(self, node, room_data, on_state_changed=None, on_history_event=None, on_replanning_event=None,
                 max_retries=2, retry_delay_seconds=2.0): # constructor del executor
        self.node = node # nodo ros 2 propietario, lo usamos para el logger, el reloj y los temporizadores
        self.room_data = room_data # diccionario nombre de habitacion -> (x, y, orientacion), para traducir el sap a coordenadas
        self.on_state_changed = on_state_changed # callback opcional: se llama cada vez que cambia gk o pk
        self.on_history_event = on_history_event # callback opcional: se llama para registrar un evento en hk
        self.on_replanning_event = on_replanning_event # callback opcional: se llama con cada evento de feedback/replanteamiento

        self.current_action = None # gk: la accion del sap que se esta ejecutando ahora, o none si esta parada
        self.pending_actions = [] # pk: cola de acciones del sap que quedan por ejecutar

        self._goal_handle = None # manejador ros 2 del objetivo en curso en nav2; permite reconocer resultados tardios
        # true desde que se envia un goal a nav2 hasta que _goal_response_callback confirma si fue aceptado o
        # rechazado. evita que una accion nueva llegue justo en ese hueco y se mande un segundo goal encima
        self._goal_in_flight = False
        self._retry_timer = None # temporizador de un solo disparo mientras esperamos para reintentar una accion fallida

        self.nav_client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses') # cliente de accion para enviar objetivos a nav2

        self.feedback_manager = ExecutionFeedbackManager() # tarea 1: monitoriza el resultado de cada accion ejecutada
        self.replanning_policy = ReplanningPolicy(max_retries=max_retries, retry_delay_seconds=retry_delay_seconds) # tarea 2: politica de reintentos

    @property
    def full_plan(self): # sap completo en orden: la accion activa (si la hay) seguida de las pendientes
        plan = []
        if self.current_action is not None:
            plan.append(self.current_action)
        plan.extend(self.pending_actions)
        return plan

    def enqueue(self, actions, discard): # ejecucion del sap: incorpora nuevas acciones, descartando o acumulando lo que hubiera
        if discard: # el usuario quiere descartar lo pendiente y quedarse solo con las nuevas acciones
            if self.current_action is not None: # si habia una accion en curso (aceptada por nav2 o todavia esperando respuesta)
                self.current_action.status = ActionStatus.FAILED # se pierde, no se retoma (no existe un estado "cancelada")
                self._notify_history(f"descartada -> {self.current_action.target}")
                self._goal_handle = None # olvidamos el manejador; nav2 sustituira el objetivo solo al llegar el nuevo (preemption)
                self._goal_in_flight = False # el envio de mas abajo sera el unico goal en vuelo a partir de ahora
                self.current_action = None
            self.pending_actions = list(actions) # pk: la cola se sustituye por completo por las nuevas acciones
        else: # el usuario quiere acumular: interrumpir la actual, hacer las nuevas antes, y retomarla despues
            self.pending_actions = list(actions) + self.pending_actions # pk: las nuevas acciones van al principio de la cola
            if self.current_action is not None: # si efectivamente habia una accion activa que interrumpir
                self._goal_handle = None # olvidamos el manejador; nav2 sustituira el objetivo solo al llegar el nuevo (preemption)
                self._goal_in_flight = False # el envio de mas abajo sera el unico goal en vuelo a partir de ahora
                self.current_action.status = ActionStatus.PENDING # vuelve a pendiente: se reintentara entera mas tarde
                self.pending_actions.append(self.current_action) # gk -> pk: al final de la cola, se retoma despues de todo lo nuevo
                self._notify_history(f"interrumpida y guardada para retomar -> {self.current_action.target}")
                self.current_action = None

        self._send_next_action() # arrancamos la primera accion de la cola (la recien anadida)
        self._notify_state_changed()

    def _send_next_action(self): # coge la primera accion pendiente de la cola y la envia a nav2
        if self._goal_handle is not None or self._goal_in_flight or self._retry_timer is not None: # viaje en curso o esperando para reintentar: no lanzamos otro encima
            return # esperamos a que termine/se confirme el actual, o a que pase el retraso del reintento; al llegar se llamara de nuevo a esta funcion

        if not self.pending_actions: # si la cola de pendientes esta vacia
            self.node.get_logger().info("Cola vacia: la silla ha llegado a su destino final.") # avisamos que no quedan acciones
            self.current_action = None # gk: no hay accion activa
            return

        next_action = self.pending_actions.pop(0) # pk: sacamos la primera accion pendiente de la cola (y la quitamos de ella)
        next_action.status = ActionStatus.IN_PROGRESS
        self.current_action = next_action # pk -> gk: esa accion pasa a ser la activa
        self._goal_in_flight = True # marcamos que ya hay un envio en curso antes de que nav2 confirme la aceptacion
        self.node.get_logger().info(f"Accion activa (Gk): {next_action.target}.") # log claro para los experimentos
        self._send_nav2_goal(next_action)

    def _send_nav2_goal(self, action): # traduce la accion goto en un objetivo de nav2 y lo envia al servidor
        self.node.get_logger().info("Esperando al servidor de accion 'NavigateThroughPoses'...") # avisamos que esperamos al servidor
        self.nav_client.wait_for_server() # bloqueamos hasta que el servidor de nav2 este disponible

        x, y, theta = self.room_data[action.target] # traducimos el objetivo semantico a coordenadas del mapa

        goal_msg = NavigateThroughPoses.Goal() # creamos el mensaje del objetivo vacio
        pose = PoseStamped() # creamos un mensaje de pose vacio
        pose.header.frame_id = 'map' # indicamos que la pose esta referida al mapa
        pose.header.stamp = self.node.get_clock().now().to_msg() # ponemos la marca de tiempo actual
        pose.pose.position.x = float(x) # fijamos la coordenada x del punto
        pose.pose.position.y = float(y) # fijamos la coordenada y del punto
        pose.pose.position.z = 0.0 # la altura es cero porque navegamos en 2d

        # magia matematica: convertir grados a cuaternion
        yaw_rad = math.radians(theta) # pasamos la orientacion de grados a radianes
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0) # calculamos la componente z del cuaternion
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0) # calculamos la componente w del cuaternion

        goal_msg.poses.append(pose) # anadimos la pose a la lista de puntos del objetivo

        self.node.get_logger().info(
            f"Enviando destino a Nav2: {action.target}. Quedan en cola (Pk): {len(self.pending_actions)}."
        ) # avisamos del envio
        send_goal_future = self.nav_client.send_goal_async(goal_msg) # enviamos el objetivo a nav2
        send_goal_future.add_done_callback(self._goal_response_callback) # registramos el callback de respuesta

    def _goal_response_callback(self, future): # callback que se ejecuta cuando nav2 acepta o rechaza el objetivo
        self._goal_in_flight = False # la respuesta ya llego: deja de estar "en vuelo" (aceptado o rechazado, cualquiera de los dos)
        goal_handle = future.result() # obtenemos el manejador del objetivo enviado

        if not goal_handle.accepted: # si nav2 ha rechazado el objetivo
            rejected_name = self.current_action.target if self.current_action is not None else "desconocido" # nombre para el log
            self.node.get_logger().error(f"La meta fue rechazada por Nav2: {rejected_name}.") # avisamos del rechazo
            self._handle_action_failure(FailureReason.REJECTED_BY_NAV2) # tareas 1+2: registramos el fallo y decidimos que hacer
            return

        self.node.get_logger().info('Meta aceptada por Nav2. Robot en movimiento.') # avisamos que nav2 acepto el destino
        self._goal_handle = goal_handle # guardamos la meta actual para poder reconocer resultados tardios
        result_future = goal_handle.get_result_async() # solicitamos el resultado de forma asincrona
        result_future.add_done_callback(
            lambda f, gh=goal_handle: self._reached_goal_callback(f, gh)
        ) # registramos el callback de llegada, ligado a este objetivo concreto

    def _reached_goal_callback(self, future, goal_handle): # callback que se ejecuta cuando el resultado de un objetivo esta disponible
        if goal_handle is not self._goal_handle: # si este resultado ya no corresponde a la accion que seguimos persiguiendo
            # la accion fue sustituida por una orden mas reciente; ignoramos su resultado tardio para no avanzar la cola dos veces
            self.node.get_logger().info("Resultado tardio de una accion ya sustituida; se ignora.")
            return

        status = future.result().status # estado final del objetivo: succeeded, canceled o aborted
        reached_name = self.current_action.target if self.current_action is not None else "desconocido" # nombre para el log y hk

        if status == GoalStatus.STATUS_SUCCEEDED: # la silla llego de verdad al destino
            self.node.get_logger().info(f"Destino alcanzado: {reached_name}.") # avisamos que la silla ha llegado
            if self.current_action is not None:
                self.current_action.status = ActionStatus.COMPLETED
                self.feedback_manager.record_success(self.current_action) # tarea 1: registramos el exito
            self._notify_history(f"completado -> {reached_name}")
            self.current_action = None # gk: la accion ya no es la activa (llegamos)
            self._goal_handle = None # marcamos que ya no hay viaje en curso
            self._send_next_action() # lanzamos la siguiente accion de la cola; aqui es donde se retoma lo interrumpido (pk -> gk)
            self._notify_state_changed()
            return

        self.node.get_logger().warn(f"El objetivo a '{reached_name}' termino sin exito (status={status}).") # avisamos del fallo
        self._handle_action_failure(FailureReason.NOT_REACHED) # tareas 1+2: registramos el fallo y decidimos que hacer

    def _handle_action_failure(self, reason): # tareas 1-4: centraliza que hacer cuando una accion del sap falla
        action = self.current_action # la accion que acaba de fallar
        if action is None: # salvaguarda: no deberia pasar, pero evitamos romper si no hay accion activa
            self._send_next_action()
            self._notify_state_changed()
            return

        outcome = self.feedback_manager.record_failure(action, reason) # tarea 1: registramos el resultado (fallo + motivo)
        decision = self.replanning_policy.decide(outcome) # tarea 2: decidimos que politica aplicar

        self.current_action = None # gk: la accion (se reintente o no) deja de ser la activa por ahora
        self._goal_handle = None

        if decision == ReplanningDecision.RETRY: # tarea 2: reintentar la accion, con un breve retraso
            action.retry_count += 1 # contamos este reintento
            action.status = ActionStatus.PENDING # vuelve a pendiente: se ve ya en pk mientras esperamos el retraso
            self.pending_actions.insert(0, action) # la primera de la cola, para reintentarla antes que cualquier otra
            self._notify_history(
                f"reintento {action.retry_count}/{self.replanning_policy.max_retries} en "
                f"{self.replanning_policy.retry_delay_seconds}s ({reason.value}) -> {action.target}"
            ) # tarea 2: registramos la decision en hk
            self._notify_replanning_event(outcome, decision) # tarea 4: publicamos el evento de feedback + replanteamiento
            self._notify_state_changed() # tarea 3: republicamos gk/pk/sap ya (gk vacio, la accion visible en pk)
            self._schedule_retry() # arrancamos el temporizador de un solo disparo; al saltar, _send_next_action la recupera
            return

        # decision == ReplanningDecision.GIVE_UP: nos rendimos con esta accion y seguimos con la siguiente del plan
        action.status = ActionStatus.FAILED
        self._notify_history(
            f"fallo definitivo tras {action.retry_count} reintento(s) ({reason.value}) -> {action.target}"
        ) # tarea 2: registramos la decision en hk
        self._notify_replanning_event(outcome, decision) # tarea 4: publicamos el evento de feedback + replanteamiento
        self._send_next_action() # seguimos con la siguiente accion de la cola
        self._notify_state_changed() # tarea 3: republicamos gk/pk/sap tras la decision

    def _schedule_retry(self): # arranca un temporizador de un solo disparo: al saltar, deja que _send_next_action recupere la accion
        def fire(): # se ejecuta una unica vez, cuando pasa el retraso configurado en la politica de replanteamiento
            self._retry_timer.cancel() # temporizador de un solo disparo: se cancela nada mas saltar (mismo patron que main.py)
            self._retry_timer = None
            self._send_next_action() # ahora si: recupera la accion (esta la primera de pk) y la reenvia a nav2
            self._notify_state_changed()
        self._retry_timer = self.node.create_timer(self.replanning_policy.retry_delay_seconds, fire)

    def _notify_replanning_event(self, outcome, decision): # avisa de un evento de feedback+replanteamiento, para publicarlo
        if self.on_replanning_event is not None:
            event = { # combinamos el resultado con la decision tomada, para verlo junto en /replanning_events
                **outcome.to_dict(),
                "decision": decision.value,
                "retry_count": outcome.action.retry_count,
                "max_retries": self.replanning_policy.max_retries,
            }
            self.on_replanning_event(event)

    def _notify_history(self, text): # registra un evento en hk si el nodo definio el callback correspondiente
        if self.on_history_event is not None:
            self.on_history_event(text)

    def _notify_state_changed(self): # avisa de que gk/pk cambiaron, para que el nodo republique su estado
        if self.on_state_changed is not None:
            self.on_state_changed()
