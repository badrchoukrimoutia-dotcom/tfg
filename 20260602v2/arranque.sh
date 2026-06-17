#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting simulation"

echo "Setting up the robot's initial conditions"
# solo el tf estatico map -> odom que coloca a la silla en el centro de la cocina; el odom -> base_link lo publica el simulador
ros2 run tf2_ros static_transform_publisher 1.5 2.0 0 0 0 0 map odom &
sleep 1

echo "Launching the wheelchair simulator"
python3 "$SCRIPT_DIR/wheelchair_simulator.py" &
sleep 2

echo "Launching the cost injection node"
python3 "$SCRIPT_DIR/main.py" &
sleep 2

echo "Starting planner and controller..."
ros2 launch nav2_bringup navigation_launch.py params_file:="$SCRIPT_DIR/mi_configuracion.yaml" use_sim_time:=False &
sleep 5

echo "Launching the semantic navigator (LLM)"
python3 "$SCRIPT_DIR/navigation_node.py" &
sleep 1

echo "Opening RViz2"
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz &

echo "Startup complete. Press Ctrl+C to stop the program."

wait
