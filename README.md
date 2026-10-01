# create3_dock_solution

Solución al [Create 3 Dock Challenge](https://github.com/Kalman-Robotics/create3_dock_challenge):
acopla un iRobot Create 3 a su dock usando solo el LiDAR (`/scan`), `/odom`, `/tf`,
`/dock_status` y `/hazard_detection`, comandando `/cmd_vel`.

- **Percepción** (`detector.py`, `perception/`): RANSAC de pared con varias candidatas,
  validación geométrica de las dos cajas marcadoras, refinamiento ICP de cerca y filtro temporal
  de la pose del dock en `odom`.
- **Control** (`controller.py`, `control/`): máquina de estados SEARCH → GO_TO_PREDOCK →
  ALIGN_AT_PREDOCK → FINAL_APPROACH → DOCKED con recuperación (BACK_OFF, reintentos). Termina
  cuando `/dock_status` reporta `is_docked: true`.
- Un solo nodo, `dock_node` (`dock_lidar`). Todos los parámetros están en `config/*.yaml`.

## Compilar

ROS 2 Humble + Gazebo Classic 11 (ver `docker/`):

```bash
cd ~/ros2/dock_ws        # workspace con src/create3_dock_challenge y src/create3_dock_solution
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Ejecutar

```bash
# Simulación
ros2 launch create3_dock_challenge challenge_world.launch.py
ros2 launch create3_dock_solution dock.launch.py

# Robot real (no hay /clock)
ros2 launch create3_dock_solution dock.launch.py use_sim_time:=false
```

Argumentos: `use_sim_time` (default `true`), `approach_mode` (`point_align` | `polar`),
`enable_controller`, `enable_markers`, `run_report`, `report_dir`, `tag`
(`ros2 launch create3_dock_solution dock.launch.py --show-args`).

## Tests

```bash
PYTHONPATH=. python3 -m pytest test/ -q
```
