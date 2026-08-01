#!/bin/bash

# carpeta donde vive este script; asi los nodos se lanzan SIEMPRE desde la carpeta correcta,
# sin importar como se llame ni donde este (antes estaba hardcodeado a inyeccion_costes_def y por
# eso se ejecutaban los archivos de la carpeta antigua en vez de los que se estaban editando)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Starting simulation"

echo "Setting up the robot's initial conditions"
# solo el tf estatico map -> odom que coloca a la silla en el centro de la cocina; el odom -> base_link lo publica el simulador
ros2 run tf2_ros static_transform_publisher 1.0 0.8 0 0 0 0 map odom &sleep 1

echo "Launching the wheelchair simulator"
python3 "$SCRIPT_DIR/wheelchair_simulator.py" &
sleep 2

echo "Launching the cost injection node"
python3 "$SCRIPT_DIR/main.py" &
sleep 2

echo "Starting planner and controller..."
# usamos NUESTRO navigation_launch.py propio (no el de nav2_bringup), porque el nuestro expone
# bond_timeout como argumento real y evita que el lifecycle_manager resetee todo el stack de nav2
# cada pocos segundos. el de nav2_bringup ignora el bond_timeout del yaml y por eso reiniciaba solo.
NAV2_LOG="/tmp/narel_nav2_launch_$$.log" # log temporal solo para detectar cuando nav2 esta listo
ros2 launch "$SCRIPT_DIR/navigation_launch.py" params_file:="$SCRIPT_DIR/mi_configuracion.yaml" use_sim_time:=False bond_timeout:=10.0 > "$NAV2_LOG" 2>&1 &

# esperamos a que el lifecycle_manager confirme que TODOS los nodos de nav2 estan activos, en vez
# de un sleep fijo a ciegas: con un sleep fijo, si el ordenador va mas lento un dia (o el modelo del
# lidar/costmap tarda un poco mas), rviz2 se abre antes de que el global_costmap haya publicado su
# primer mapa y aparece vacio hasta que lo reseteas a mano.
echo "Esperando a que Nav2 termine de activarse..."
until grep -q "Managed nodes are active" "$NAV2_LOG" 2>/dev/null; do
    sleep 0.5
done
sleep 2 # margen extra para que el global_costmap complete al menos un ciclo de publicacion (1 hz)
echo "Nav2 activo."

echo "Launching the semantic navigator (LLM)"
python3 "$SCRIPT_DIR/navigation_node.py" &
sleep 1

echo "Opening RViz2"
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz &

echo "Startup complete. Press Ctrl+C to stop the program."

wait