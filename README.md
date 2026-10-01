# Acomplamiento con percpeción con LiDAR al dock
 
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

> Estado: 🚧 en desarrollo — estructura del paquete y entorno Docker listos; nodo de docking pendiente.
 
## Equipo: NorthDock
 
| Nombre | Correo |
|---|---|
| Esaú Arqueros | esau.arqueros.24@gmail.com |
| Elias Cabeza | jose.cabeza@utec.edu.pe |
 
## Qué contiene este repo
 
```
create3_dock_solution/
├── package.xml               # paquete ROS 2 (ament_python) y sus dependencias
├── setup.py / setup.cfg      # instalación del paquete Python
├── resource/                 # marcador del índice de paquetes de ament
├── create3_dock_solution/    # código del nodo (en desarrollo)
├── test/                     # tests de estilo (flake8, pep257, copyright)
└── docker/
    ├── Dockerfile            # Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + create3_sim
    ├── ros_env.sh            # entorno ROS automático en cada terminal del contenedor
    └── docker-compose.yml    # levanta el contenedor con GUI y el workspace montado
```
 
Este repo es **solo nuestro paquete**. El paquete del reto (`create3_dock_challenge`) se clona
aparte en el mismo workspace y **no se modifica**.
 
## Estructura del workspace
 
```
~/ros2/dock_ws/
└── src/
    ├── create3_dock_challenge/   # repo del reto (sin tocar)
    └── create3_dock_solution/    # este repo
```


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

 
## Puesta en marcha
 
Requisitos: Docker Engine con el plugin `docker compose`. El host puede ser cualquier Ubuntu
(p. ej. 24.04 con Jazzy): el contenedor aporta Humble y Gazebo Classic.
 
```bash
# 1. Workspace y clones
mkdir -p ~/ros2/dock_ws/src && cd ~/ros2/dock_ws/src
git clone https://github.com/Kalman-Robotics/create3_dock_challenge.git
git clone git@github.com:esauarqueros/create3-dock-solution.git create3_dock_solution
 
# 2. Construir y levantar el contenedor
cd create3_dock_solution/docker
export HOST_UID=$(id -u) HOST_GID=$(id -g)
xhost +local:docker
docker compose build
docker compose up -d
docker compose exec dev bash
 
# 3. Dentro del contenedor: compilar y lanzar el escenario
cd /ws && colcon build --symlink-install
source install/setup.bash
ros2 launch create3_dock_challenge challenge_world.launch.py
```
 
Para detener el contenedor: `docker compose down` (desde `docker/`).
 
## Cómo contribuir
 
1. Parte siempre de `main` actualizado: `git switch main && git pull`.
2. Crea una rama por tarea: `git switch -c feat/<tarea>`.
3. Commit y push de la rama: `git push -u origin feat/<tarea>`.
4. Abre un Pull Request hacia `main` y pide revisión.
No subir `build/`, `install/` ni `log/`. No hardcodear poses ni coordenadas del mundo (descalifica).
 
## Comando de lanzamiento de la solución
 
_(Pendiente — será `ros2 launch create3_dock_solution solucion.launch.py`.)_
 
## Licencia
 
_(Pendiente — definir en `package.xml`; se sugiere Apache-2.0, igual que el reto.)_
 
