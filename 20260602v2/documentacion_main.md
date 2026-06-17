# Documentación de `main.py` — Nodo `CostInjectionNode`

## 1. Propósito general

`main.py` define un nodo de ROS 2 (`CostInjectionNode`) que:

1. Carga una biblioteca de costes por tipo de objeto (`costes_preestablecidos.json`) y un mapa del entorno (`ejemplo_natalia.json`).
2. Genera una matriz de costes (`numpy.ndarray`) a partir de esos datos usando `complex_cost_injection` (definida en `cost_module.py`).
3. Normaliza esa matriz a la escala que espera `nav_msgs/OccupancyGrid` (0-100) y la publica cada 2 segundos en el topic `/mapa_de_costes`, para que Nav2 la use como capa de costes.

## 2. Lógica principal, paso a paso

### 2.1 Constructor `__init__`

| Línea | Qué hace |
|---|---|
| 13 | Inicializa el nodo ROS 2 con el nombre `cost_injection_node`. |
| 15-19 | Define un `QoSProfile` con `depth=1`, `TRANSIENT_LOCAL` y `RELIABLE`, igual al que usa Nav2 para mapas estáticos: así un suscriptor que se conecte tarde (p. ej. RViz) recibe igualmente el último mapa publicado. |
| 21 | Crea el publicador del topic `/mapa_de_costes` de tipo `OccupancyGrid`. |
| 23-24 | Crea un `Timer` que llama a `publish_map` cada 2 segundos. |
| 27 | Carga la biblioteca de costes (`{objeto: coste}`) desde JSON. |
| 29 | Carga el entorno (habitaciones, conexiones, objetos) desde JSON. |
| 32-37 | Si ambos datasets se cargaron, genera la matriz de costes con `complex_cost_injection`; si no, registra un error. |

### 2.2 `publish_map` (se ejecuta cada 2 s)

| Línea | Qué hace |
|---|---|
| 40-41 | Si todavía no hay matriz generada, no publica nada. |
| 43-47 | Construye la cabecera del mensaje (`stamp`, `frame_id='map'`). |
| 49-59 | Construye los metadatos (`resolution`, `width`, `height`, `origin`) a partir de las medidas devueltas por `complex_cost_injection`. |
| 61-79 | **Normalización de escala**: convierte la matriz interna (`uint8`, 0=libre, 1-252=coste gradual, 253=inflado máximo, 254=letal) a la escala que espera Nav2/OccupancyGrid (0=libre, 1-99=coste gradual, 100=letal), tratando los valores frontera (0, 253, 254) de forma explícita y escalando linealmente el resto. |
| 81-83 | Aplana la matriz a una lista 1D (orden row-major, el que espera `OccupancyGrid.data`) y publica el mensaje. |

### 2.3 `main()`

Inicializa `rclpy`, crea el nodo, lo mantiene vivo con `rclpy.spin()` y al recibir `Ctrl+C` lo destruye y cierra `rclpy` en un bloque `finally`.

---

## 3. Errores y bugs detectados

### 🔴 3.1 — Creación del nodo fuera del bloque `try/finally` (Alto)

**Dónde:** línea 88, función `main()`.

```python
node = CostInjectionNode()  # FUERA del try

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    ...
finally:
    node.destroy_node()
    rclpy.shutdown()
```

**Problema:** si el constructor de `CostInjectionNode` lanza una excepción (por ejemplo, un `KeyError` dentro de `complex_cost_injection` por un JSON mal formado, sin la clave `position_x`, etc.), la excepción se propaga **sin pasar por el `finally`**, porque la llamada está fuera del `try`. Esto significa que `rclpy.shutdown()` nunca se ejecuta: el contexto de `rclpy` queda inicializado pero "huérfano", lo que puede provocar errores en llamadas posteriores dentro del mismo proceso (tests, reintentos, etc.) y deja un traceback crudo en lugar de un cierre controlado.

La documentación oficial de Python es explícita sobre esta garantía: la cláusula `finally` solo se ejecuta para excepciones lanzadas **dentro** del `try` ([Defining Clean-up Actions](https://docs.python.org/3/tutorial/errors.html#defining-clean-up-actions); [The `try` statement](https://docs.python.org/3/reference/compound_stmts.html#the-try-statement)).

**Cambio sugerido:**

```python
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CostInjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
```

---

### 🔴 3.2 — Rutas de archivo relativas dependientes del directorio de trabajo (Alto)

**Dónde:** líneas 27 y 29.

```python
self.cost_library = load_cost_library('costes_preestablecidos.json')
self.detected_objects = load_environment_json('ejemplo_natalia.json')
```

**Problema:** según la documentación oficial de `open()`, una ruta relativa se resuelve contra el **directorio de trabajo actual del proceso**, no contra la ubicación del script ([`open()` — Python docs](https://docs.python.org/3/library/functions.html#open)). Cuando este nodo se lanza con `ros2 run` o desde un *launch file*, el directorio de trabajo casi nunca coincide con la carpeta donde está `main.py`. El fallo se "traga" silenciosamente porque `load_cost_library`/`load_environment_json` capturan la excepción y devuelven `{}`/`None`, así que el único síntoma es un `error` en el log y el nodo publicando nada, sin más contexto.

**Cambio sugerido** (usar `pathlib`, recomendado en la [documentación de `pathlib`](https://docs.python.org/3/library/pathlib.html) para construir rutas robustas):

```python
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

self.cost_library = load_cost_library(BASE_DIR / 'costes_preestablecidos.json')
self.detected_objects = load_environment_json(BASE_DIR / 'ejemplo_natalia.json')
```

> Nota ROS 2: si estos JSON se instalan como `share` del paquete, lo más correcto a largo plazo es resolverlos con `ament_index_python.packages.get_package_share_directory(...)` en vez de una ruta relativa al `.py`.

---

### 🟠 3.3 — Comprobación por veracidad (*truthiness*) en vez de `is not None` (Medio)

**Dónde:** línea 32.

```python
if self.detected_objects and self.cost_library:
```

**Problema:** `load_cost_library` devuelve `{}` cuando falla la carga (línea 11 de `cost_module.py`), y `{}` es *falsy* en Python — pero un diccionario vacío también sería *falsy* si el JSON se cargó **correctamente** pero estaba vacío (p. ej. una biblioteca de costes sin entradas todavía). En ese caso el código entraría por error en la rama de "Missing data" aunque la carga fuese exitosa. Lo mismo aplica si `detected_objects` fuese un entorno válido pero sin habitaciones (`{}`).

La [PEP 8 — Programming Recommendations](https://peps.python.org/pep-0008/#programming-recommendations) indica que las comparaciones contra singletons como `None` deben hacerse siempre con `is`/`is not`, precisamente para no confundir "vacío" con "ausente".

**Cambio sugerido:**

```python
if self.detected_objects is not None and self.cost_library is not None:
```

(y hacer que `load_cost_library` devuelva `None` en caso de error en vez de `{}`, para que la semántica de "fallo" sea consistente).

---

### 🟠 3.4 — `width`/`height` recalculados de forma independiente, con la resolución duplicada en dos módulos (Medio)

**Dónde:** líneas 50, 53-54 de `main.py`, y línea 14 de `cost_module.py`.

```python
# main.py
msg.info.resolution = 0.1
msg.info.width  = int(self.map_w / msg.info.resolution)
msg.info.height = int(self.map_h / msg.info.resolution)
```

```python
# cost_module.py
resolution = 0.1
...
rows = int(height_m / resolution)
cols = int(width_m / resolution)
costmap = np.zeros((rows, cols), dtype=np.uint8)
```

**Problema:** la constante `0.1` está duplicada en dos archivos en lugar de tener una única fuente de verdad. Si en el futuro se cambia la resolución en un solo sitio (algo muy fácil de hacer por error), `msg.info.width * msg.info.height` dejará de coincidir con el tamaño real de `self.costmap` (y por tanto con `len(msg.data)`), produciendo un `OccupancyGrid` corrupto que puede hacer fallar o renderizar mal en RViz/Nav2. Además, recalcular `width`/`height` mediante una división de punto flotante es una fuente de verdad redundante cuando el array ya tiene la forma exacta.

**Cambio sugerido:** usar directamente la forma del array, que es la única fuente de verdad real:

```python
msg.info.width = self.costmap.shape[1]
msg.info.height = self.costmap.shape[0]
```

Y, si se quiere mantener `resolution` como parámetro configurable, declararlo una sola vez (idealmente como parámetro ROS 2 con `self.declare_parameter('resolution', 0.1)`) y pasarlo a `complex_cost_injection` en vez de tenerlo hardcodeado en dos sitios distintos.

---

### 🟡 3.5 — `np.empty_like` para el buffer de salida (Bajo, pero frágil)

**Dónde:** línea 67.

```python
data = self.costmap.astype(np.int16)
output = np.empty_like(data)
```

**Problema:** la documentación oficial de NumPy es explícita: *"This function \[`empty`\] does not initialize the returned array; to do that use `zeros` instead"* ([`numpy.empty` — NumPy docs](https://numpy.org/doc/stable/reference/generated/numpy.empty.html)). Ahora mismo el código cubre exactamente los rangos `{0}`, `{253}`, `{254}` y `[1, 252]`, que en conjunto son todo el dominio válido de `uint8` usado (0-254), por lo que no hay bug activo hoy. Pero es una construcción frágil: si en el futuro se introduce algún valor no cubierto por las máscaras (p. ej. un 255 para "desconocido"), el mensaje publicado contendría **memoria sin inicializar** filtrada a la red ROS 2, lo cual es mucho peor que un valor incorrecto pero determinista.

**Cambio sugerido:**

```python
output = np.zeros_like(data)
```

El coste de rendimiento de `zeros_like` frente a `empty_like` es despreciable comparado con el riesgo de publicar memoria basura.

---

### 🟡 3.6 — Atributos definidos solo dentro de la rama `if` (Bajo)

**Dónde:** líneas 31-37.

```python
self.costmap = None
if self.detected_objects and self.cost_library:
    self.costmap, self.map_w, self.map_h, self.origin_x, self.origin_y = complex_cost_injection(...)
    ...
else:
    self.get_logger().error('Missing data. Check the JSON files.')
```

**Problema:** `self.map_w`, `self.map_h`, `self.origin_x` y `self.origin_y` solo existen como atributos si la rama `if` se ejecuta. Hoy esto no rompe nada porque `publish_map` corta antes (`if self.costmap is None: return`), pero es un patrón frágil: cualquier refactor futuro que toque ese `guard` provocará un `AttributeError` en tiempo de ejecución. La [documentación de Python sobre clases](https://docs.python.org/3/tutorial/classes.html) recomienda que los atributos de instancia queden definidos de forma predecible en `__init__`, independientemente de la rama tomada.

**Cambio sugerido:**

```python
self.costmap = None
self.map_w = self.map_h = self.origin_x = self.origin_y = None
if self.detected_objects is not None and self.cost_library is not None:
    ...
```

---

### 🟡 3.7 — El nodo queda "vivo" indefinidamente sin publicar si fallan los datos (Bajo / diseño)

**Dónde:** líneas 36-37.

**Problema:** si el JSON de costes o de entorno no se cargan, el nodo registra un único `error` al arrancar y luego sigue funcionando para siempre (el timer sigue llamando a `publish_map`, que simplemente retorna sin hacer nada). Desde fuera, el nodo parece "vivo y sano" (aparece en `ros2 node list`), pero nunca publicará nada y no hay reintento ni mecanismo de recuperación. No es un bug en el sentido estricto, pero conviene decidir explícitamente el comportamiento deseado: reintentar la carga, o cerrar el nodo con un código de salida distinto de cero para que el supervisor/launch system lo note.

---

## 4. Resumen de cambios sugeridos (prioridad)

1. Mover la creación de `CostInjectionNode()` dentro del `try` en `main()` para garantizar `rclpy.shutdown()` siempre (3.1).
2. Resolver las rutas de los JSON con `pathlib.Path(__file__).resolve().parent` en vez de rutas relativas al cwd (3.2).
3. Cambiar las comprobaciones de carga a `is not None` (3.3).
4. Calcular `msg.info.width`/`height` desde `self.costmap.shape` en vez de recomputar con `resolution` duplicada (3.4).
5. Sustituir `np.empty_like` por `np.zeros_like` en la normalización (3.5).
6. Inicializar todos los atributos de instancia en `__init__`, también en la rama de error (3.6).
7. Decidir y documentar el comportamiento del nodo cuando faltan los datos de entrada (3.7).
