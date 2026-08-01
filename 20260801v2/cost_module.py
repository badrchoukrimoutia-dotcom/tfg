import json # importamos la libreria para leer el repositorio de costes en formato json
import numpy as np # importamos numpy para construir y manipular la matriz del mapa de forma eficiente
import matplotlib.pyplot as plt # importamos matplotlib para poder generar la imagen de depuracion del mapa

# probado en real: con 253 (inscribed inflated obstacle) el planificador (SmacPlannerHybrid, cost_penalty: 2.0
# en mi_configuracion.yaml) solo lo trata como una penalizacion, no como un bloqueo real, asi que si es la unica
# ruta posible la usa igualmente aunque sea cara. usamos 254 (letal), el mismo valor que las paredes reales, que
# si es infranqueable para cualquier planificador de nav2 sin ambiguedad: una restriccion debe poder prohibir de
# facto, a diferencia de una preferencia (ver SLOW_COST) que solo debe desincentivar.
AVOID_COST = 254 # coste letal para las zonas restringidas: nav2 nunca planifica a traves de ellas, igual que una pared real
SLOW_COST = 130 # coste moderado para las zonas con preferencia de velocidad reducida: desincentiva la zona pero no la prohibe

def _meters_to_indices(x, y, offset_x, offset_y, resolution): # convierte una coordenada en metros a indices de fila/columna de la matriz
    col = int((x + offset_x) / resolution) # convertimos la coordenada x en numero de columna
    row = int((y + offset_y) / resolution) # convertimos la coordenada y en numero de fila
    return row, col # devolvemos la posicion dentro de la matriz

def load_cost_library(cost_path): # funcion que carga el diccionario con el coste asignado a cada tipo de objeto
    try: # intentamos abrir el archivo de costes de forma segura
        with open(cost_path, 'r', encoding='utf-8') as f: # abrimos el json de costes en modo lectura
            return json.load(f) # devolvemos el diccionario con todos los costes preestablecidos
    except Exception as e: # si la lectura falla capturamos la excepcion
        print(f"Error reading the cost library: {e}") # informamos por terminal del error ocurrido
        return {} # devolvemos un diccionario vacio para que el programa no se rompa

def complex_cost_injection(environment_data, cost_library, debug_plot=False): # funcion principal que convierte el json arquitectonico en la matriz del mapa de costes
    resolution = 0.1 # definimos que cada celda de la matriz equivale a 10 cm en el mundo real

    # =================================================================
    # FASE 1: CALCULAR EL LIENZO AUTOMATICO (BOUNDING BOX)
    # =================================================================
    min_x, min_y = float('inf'), float('inf') # inicializamos los limites minimos muy altos para irlos reduciendo
    max_x, max_y = float('-inf'), float('-inf') # inicializamos los limites maximos muy bajos para irlos aumentando

    for room in environment_data.get('rooms', []): # recorremos todas las habitaciones para encontrar los bordes de la casa
        rx = room['position_x'] # leemos la coordenada x de la esquina de la habitacion
        ry = room['position_y'] # leemos la coordenada y de la esquina de la habitacion
        rw = room['width'] # leemos el ancho de la habitacion
        rh = room['height'] # leemos el alto de la habitacion

        if rx < min_x: min_x = rx # si esta habitacion empieza mas a la izquierda actualizamos el minimo en x
        if ry < min_y: min_y = ry # si esta habitacion empieza mas abajo actualizamos el minimo en y
        if (rx + rw) > max_x: max_x = (rx + rw) # si esta habitacion termina mas a la derecha actualizamos el maximo en x
        if (ry + rh) > max_y: max_y = (ry + rh) # si esta habitacion termina mas arriba actualizamos el maximo en y

    margin = 1.0 # definimos un metro de margen para que la casa no toque los bordes del mapa
    min_x -= margin # aplicamos el margen al limite izquierdo
    min_y -= margin # aplicamos el margen al limite inferior
    max_x += margin # aplicamos el margen al limite derecho
    max_y += margin # aplicamos el margen al limite superior

    width_m = max_x - min_x # calculamos el ancho total del mapa en metros
    height_m = max_y - min_y # calculamos el alto total del mapa en metros

    offset_x = -min_x # calculamos el desplazamiento en x para convertir coordenadas negativas en indices positivos
    offset_y = -min_y # calculamos el desplazamiento en y para convertir coordenadas negativas en indices positivos

    rows = int(height_m / resolution) # calculamos cuantas filas tendra la matriz a partir del alto y la resolucion
    cols = int(width_m / resolution) # calculamos cuantas columnas tendra la matriz a partir del ancho y la resolucion
    costmap = np.zeros((rows, cols), dtype=np.uint8) # creamos el lienzo vacio lleno de ceros (espacio libre)

    def m_to_idx(x, y): # funcion auxiliar que pasa de metros (x, y) a indices de la matriz (fila, columna)
        return _meters_to_indices(x, y, offset_x, offset_y, resolution) # reutilizamos la misma conversion que usa apply_semantic_overlay

    # =================================================================
    # FASE 2: LEVANTAR LOS MUROS PERIMETRALES DE LAS HABITACIONES
    # =================================================================
    for room in environment_data.get('rooms', []): # recorremos cada habitacion para dibujar sus cuatro paredes
        rx = room['position_x'] # leemos la x de la esquina de la habitacion
        ry = room['position_y'] # leemos la y de la esquina de la habitacion
        rw = room['width'] # leemos el ancho de la habitacion
        rh = room['height'] # leemos el alto de la habitacion

        r_bottom, c_left = m_to_idx(rx, ry) # calculamos la esquina inferior izquierda en indices
        r_top, c_right = m_to_idx(rx + rw, ry + rh) # calculamos la esquina superior derecha en indices

        # el valor 254 representa un obstaculo letal (pared infranqueable)
        costmap[r_bottom:r_top, c_left] = 254 # dibujamos la pared izquierda (oeste)
        costmap[r_bottom:r_top, c_right-1] = 254 # dibujamos la pared derecha (este)
        costmap[r_bottom, c_left:c_right] = 254 # dibujamos la pared inferior (sur)
        costmap[r_top-1, c_left:c_right] = 254 # dibujamos la pared superior (norte)

    # =================================================================
    # FASE 3: ABRIR LOS HUECOS DE LAS PUERTAS (CONEXIONES)
    # =================================================================
    rooms_dict = {room['name']: room for room in environment_data.get('rooms', [])} # creamos un diccionario para localizar cada habitacion por su nombre

    for conn in environment_data.get('connections', []): # recorremos todas las conexiones (puertas) entre habitaciones
        if conn['origin_room'] in rooms_dict: # comprobamos que la habitacion de origen existe en el diccionario
            room = rooms_dict[conn['origin_room']] # recuperamos los datos de la habitacion de origen
            rx = room['position_x'] # leemos la x de la habitacion de origen
            ry = room['position_y'] # leemos la y de la habitacion de origen

            w_pos = conn['wall_position'] # leemos en que pared esta la puerta (norte, sur, este, oeste)
            w_offset = conn['wall_offset'] # leemos a cuantos metros de la esquina empieza la puerta
            w_width = conn['width'] # leemos el ancho de la puerta

            if w_pos == 'north': # si la puerta esta en la pared norte
                hole_start_r, hole_start_c = m_to_idx(rx + w_offset, ry + room['height']) # calculamos el inicio del hueco
                _, hole_end_c = m_to_idx(rx + w_offset + w_width, ry + room['height']) # calculamos el final del hueco
                costmap[hole_start_r-1:hole_start_r+1, hole_start_c:hole_end_c] = 0 # abrimos el hueco poniendo las celdas a libre

            elif w_pos == 'south': # si la puerta esta en la pared sur
                hole_start_r, hole_start_c = m_to_idx(rx + w_offset, ry) # calculamos el inicio del hueco
                _, hole_end_c = m_to_idx(rx + w_offset + w_width, ry) # calculamos el final del hueco
                costmap[hole_start_r-1:hole_start_r+1, hole_start_c:hole_end_c] = 0 # abrimos el hueco de la puerta

            elif w_pos == 'east': # si la puerta esta en la pared este
                hole_start_r, hole_start_c = m_to_idx(rx + room['width'], ry + w_offset) # calculamos el inicio del hueco
                hole_end_r, _ = m_to_idx(rx + room['width'], ry + w_offset + w_width) # calculamos el final del hueco
                costmap[hole_start_r:hole_end_r, hole_start_c-1:hole_start_c+1] = 0 # abrimos el hueco de la puerta

            elif w_pos == 'west': # si la puerta esta en la pared oeste
                hole_start_r, hole_start_c = m_to_idx(rx, ry + w_offset) # calculamos el inicio del hueco
                hole_end_r, _ = m_to_idx(rx, ry + w_offset + w_width) # calculamos el final del hueco
                costmap[hole_start_r:hole_end_r, hole_start_c-1:hole_start_c+1] = 0 # abrimos el hueco de la puerta

    # =================================================================
    # FASE 4: COLOCAR LOS OBJETOS / MUEBLES DENTRO
    # =================================================================
    for room in environment_data.get('rooms', []): # recorremos cada habitacion para colocar sus muebles
        for obj in room.get('objects', []): # recorremos cada objeto que hay dentro de la habitacion
            obj_class = obj["object_class"] # leemos la clase del objeto (sofa, mesa, etc.)
            cost = cost_library.get(obj_class, 254) # buscamos su coste en la biblioteca, o 254 si no esta definido
            print(f"Injecting '{obj_class}' with cost: {cost}") # avisamos por terminal que objeto estamos inyectando

            ox = obj["position_x"] # leemos la x del centro del objeto
            oy = obj["position_y"] # leemos la y del centro del objeto
            ow = obj["width"] # leemos el ancho del objeto
            oh = obj["height"] # leemos el alto del objeto

            # asumimos que las coordenadas del objeto marcan su centro geometrico
            obj_r_bottom, obj_c_left = m_to_idx(ox - ow/2, oy - oh/2) # calculamos la esquina inferior izquierda del objeto
            obj_r_top, obj_c_right = m_to_idx(ox + ow/2, oy + oh/2) # calculamos la esquina superior derecha del objeto

            costmap[obj_r_bottom:obj_r_top, obj_c_left:obj_c_right] = cost # pintamos el objeto en la matriz con su coste

    # =================================================================
    # FASE 5: EXPORTAR IMAGEN DE DEPURACION (OPCIONAL) Y DEVOLVER DATOS
    # =================================================================
    if debug_plot: # solo generamos la imagen si activamos el modo depuracion
        fig, ax = plt.subplots() # creamos una figura y unos ejes nuevos para dibujar
        im = ax.imshow(costmap, origin='lower', cmap='hot', extent=[min_x, max_x, min_y, max_y]) # dibujamos la matriz como un mapa de calor
        ax.set_xlabel("Meters (real X)") # etiquetamos el eje x en metros reales
        ax.set_ylabel("Meters (real Y)") # etiquetamos el eje y en metros reales
        fig.colorbar(im, ax=ax, label="Cost value") # anadimos una barra de color para interpretar los costes
        ax.set_title("Generated Cost Map (Automatic Bounding Box)") # ponemos un titulo descriptivo a la imagen
        fig.savefig('mapa_final.png') # guardamos la imagen en un archivo png
        plt.close(fig) # cerramos la figura para liberar la memoria
        print(f"\nImage generated: Bounding Box from ({min_x:.2f}, {min_y:.2f}) of {width_m:.2f}x{height_m:.2f} meters.") # informamos del tamano del mapa generado

    return costmap, width_m, height_m, min_x, min_y # devolvemos la matriz y todas las medidas clave que necesita ros 2

def apply_semantic_overlay(base_costmap, environment_data, min_x, min_y, resolution, constraints, preferences): # anade restricciones y preferencias activas sobre el costmap base
    # partimos siempre del costmap base (paredes+muebles) sin modificarlo, para poder recalcular cada vez que cambien las restricciones/preferencias
    costmap = base_costmap.copy() # copiamos el costmap base para no alterar el original entre recalculos
    offset_x = -min_x # mismo desplazamiento que usa complex_cost_injection para pasar de metros a indices
    offset_y = -min_y

    rooms_by_name = {room['name'].lower(): room for room in environment_data.get('rooms', [])} # localizamos cada habitacion por su nombre

    def paint_zone(room_name, cost_value): # pinta el rectangulo completo de una habitacion con un coste minimo, sin rebajar lo que ya hubiera
        room = rooms_by_name.get(room_name) # buscamos la habitacion por su nombre
        if room is None: # si la zona no existe en el mapa (ya deberia haberse filtrado en el grounding) no hacemos nada
            return
        r_bottom, c_left = _meters_to_indices(room['position_x'], room['position_y'], offset_x, offset_y, resolution) # esquina inferior izquierda
        r_top, c_right = _meters_to_indices(room['position_x'] + room['width'], room['position_y'] + room['height'], offset_x, offset_y, resolution) # esquina superior derecha
        zone = costmap[r_bottom:r_top, c_left:c_right] # recorte de la zona de esa habitacion
        costmap[r_bottom:r_top, c_left:c_right] = np.maximum(zone, cost_value) # subimos el coste sin bajar nunca el de paredes/muebles ya pintados

    for constraint in constraints: # restriccion: coste muy alto, para que nav2 evite la zona por completo al planificar
        paint_zone(constraint['target'], AVOID_COST)

    for preference in preferences: # preferencia: coste moderado, desincentiva la zona pero no la prohibe
        paint_zone(preference['target'], SLOW_COST)

    return costmap # devolvemos el costmap con las restricciones/preferencias ya aplicadas
