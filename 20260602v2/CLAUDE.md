# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ROS 2 (rclpy) simulation stack for a semantically-controlled robotic wheelchair. A house is described as JSON (rooms, doors, furniture), turned into a Nav2 costmap, and the chair is driven by natural-language commands parsed by a local LLM (Ollama) into Nav2 navigation goals. This is a loose collection of standalone Python scripts, **not** a colcon/ament package (no `package.xml` or `setup.py`) — there is no build step, no test suite, and no linter configured in this repo.

## Running the system

Everything is launched together via:

```bash
./arranque.sh
```

This starts, in order: a static TF (`map -> odom`), `wheelchair_simulator.py`, `main.py`, Nav2 bringup (`ros2 launch nav2_bringup navigation_launch.py params_file:=mi_configuracion.yaml`), `navigation_node.py`, and RViz2.

To iterate on a single node, source the ROS 2 workspace and run it directly, e.g.:

```bash
python3 main.py                 # cost injection node, publishes /mapa_de_costes
python3 wheelchair_simulator.py # kinematic sim, publishes odom->base_link TF
python3 navigation_node.py      # LLM-driven semantic navigator
```

Runtime dependencies beyond ROS 2 core: `numpy`, `matplotlib` (cost_module debug plot), `requests` (Ollama HTTP API). `navigation_node.py` additionally requires an [Ollama](https://ollama.com) server running locally on `localhost:11434` with the `phi3` model pulled (swap via `self.llm_model` in `navigation_node.py`).

**Note:** `arranque.sh` and `navigation_node.py` hardcode absolute paths to `/home/badrc/inyeccion_costes_def/...`. If this checkout lives elsewhere (it currently doesn't match this repo's actual path), those paths need updating before the stack will run end-to-end.

## Architecture

### Cost map generation pipeline (`main.py` + `cost_module.py`)

`main.py`'s `CostInjectionNode` is the bridge between the static house description and Nav2:

1. Loads `ejemplo_natalia.json` (rooms with corner position + width/height + nested furniture objects with center position + width/height, plus inter-room door `connections`) and `costes_preestablecidos.json` (a flat `{object_class: cost}` lookup, costs in the internal 0-254 scale).
2. `cost_module.complex_cost_injection()` auto-sizes a bounding box around all rooms (+1 m margin), rasterizes it into a `numpy` `uint8` grid at 0.1 m/cell, draws room perimeter walls (254 = lethal), carves door-sized gaps (0 = free) from `connections`, then paints each room's furniture at its looked-up cost (defaulting to 254 — lethal — for unknown object classes).
3. `publish_map()` (in `main.py`) rescales that internal uint8 scale (`0`=free, `1-252`=gradual, `253`=max inflation, `254`=lethal) into the `nav_msgs/OccupancyGrid` scale Nav2 expects (`0`=free, `1-99`=gradual, `100`=lethal), and publishes on `/mapa_de_costes` every 2 s with `TRANSIENT_LOCAL` + `RELIABLE` QoS so late subscribers (RViz, Nav2's static layer) still get the last map.
4. `mi_configuracion.yaml` wires `/mapa_de_costes` into both the global and local costmap as a `nav2_costmap_2d::StaticLayer` named `mapa_semantico`. The flags `trinary_costmap: False`, `track_unknown_space: False`, `use_maximum: False` are load-bearing: without them Nav2 collapses the gradual costs back down to a free/lethal binary map.

A known-bugs/code-review writeup of `main.py` already exists at [documentacion_main.md](documentacion_main.md) (covers a `try/finally` resource-leak risk in `main()`, cwd-relative JSON paths, truthy-vs-`is not None` checks, and the duplicated `resolution` constant between `main.py` and `cost_module.py`) — check it before re-auditing that file.

### Semantic navigation pipeline (`navigation_node.py`)

`SemanticNavigator` is an independent pipeline that turns spoken/typed commands into Nav2 action goals:

1. Subscribes to `/comando_usuario` (`std_msgs/String`).
2. Sends the command text to the local Ollama server with a system prompt that restricts the model to the exact room names present in `ejemplo_natalia.json` (loaded again here, independently from `main.py`/`cost_module.py`, via its own `load_map_data` which computes room centers rather than corners), forcing JSON output and handling both `{"key": [...]}` and bare-list response shapes.
3. Resolves each returned room name to `(x, y, orientation)` and manages a waypoint queue with two modes detected via `discard_keywords`: **discard** (replace the queue, cancel the in-flight goal) vs **accumulate** (new destinations are pushed to the *front* of the queue, deferring whatever was already pending).
4. Drives the queue one goal at a time through the `NavigateThroughPoses` action client (`send_nav2_goal` -> `goal_response_callback` -> `reached_goal_callback` -> `send_next_goal`), converting room orientation (degrees) to a quaternion manually (`sin/cos(yaw/2)`, no roll/pitch).

### Cross-node contracts

- `/cmd_vel_nav` (`geometry_msgs/Twist`): Nav2's `velocity_smoother` output → consumed by `wheelchair_simulator.py`'s differential-drive integrator, which publishes the `odom -> base_link` TF at 50 Hz.
- `/mapa_de_costes` (`nav_msgs/OccupancyGrid`): `main.py` → Nav2 static layer (both global and local costmap).
- `navigate_through_poses` action: `navigation_node.py` → Nav2's `bt_navigator`.
- The house layout JSON (`ejemplo_natalia.json`) is read independently by both `main.py`/`cost_module.py` (room corners, for rasterizing) and `navigation_node.py` (room centers, for goal coordinates) — there is no shared loader, so any change to the room schema must be kept consistent across both consumers.
