# Docking del Create 3 con LiDAR — NorthDock

Solución al [Create 3 Dock Challenge](https://github.com/Kalman-Robotics/create3_dock_challenge)
(HRFEST 2026): acopla un iRobot Create 3 a su dock **percibiendo el dock solo con el LiDAR**.
Lee `/scan`, `/odom`, `/tf`, `/dock_status` y `/hazard_detection`, y comanda `/cmd_vel`. No usa
la acción `/dock`, los sensores IR, el ground truth, `/gazebo/*` ni el frame `std_dock_link`.

> **Para el jurado — commit a evaluar: `d305d7d`**
> (`d305d7df60af886d38cb49a537f16b3513fb48c2`). Después de clonar:
> `git checkout d305d7d`.

## Equipo: NorthDock

| Nombre | Correo |
|---|---|
| Esaú Arqueros | esau.arqueros.24@gmail.com |
| Elias Cabeza | jose.cabeza@utec.edu.pe |

## Comando único de lanzamiento

```bash
ros2 launch create3_dock_solution dock.launch.py
```

Con el escenario del reto ya corriendo, el nodo empieza a trabajar solo y se detiene al
acoplarse (`/dock_status` → `is_docked: true`). Los valores por defecto son los evaluados:
modo de aproximación `point_align` y `use_sim_time:=true`.

## Instalación y compilación

Este repo contiene **solo nuestro paquete** (`create3_dock_solution`). El paquete del reto
(`create3_dock_challenge`) se clona aparte, en el mismo workspace, y **no se modifica**:

```
<workspace>/
└── src/
    ├── create3_dock_challenge/   # repo del reto (sin tocar)
    └── create3_dock_solution/    # este repo
```

Hay dos formas de prepararlo. Despliega la que vayas a usar:

<details>
<summary><b>Opción A — Docker (recomendada, la que usamos)</b></summary>

<br>

No hace falta tener ROS instalado: sirve en cualquier Ubuntu (p. ej. 24.04). La imagen
(`docker/Dockerfile`, basada en `osrf/ros:humble-desktop`) trae **ROS 2 Humble, Gazebo Classic 11 y
todas las dependencias** del reto (simulación del Create 3, `irobot_create_msgs`, xacro…) y de
nuestro paquete (numpy, scipy, matplotlib). Para resolver las dependencias del reto, lo clona de
forma temporal durante el build, ejecuta `rosdep` y lo borra.

La imagen **no contiene código**. El workspace (reto + solución) vive en tu máquina y se monta en
el contenedor como `/ws`. Ambos repos se clonan tal cual y el reto no se modifica.

Requisitos: Docker Engine con el plugin `docker compose`, y X11 para ver Gazebo y RViz.

```bash
# 1. Workspace y clones
mkdir -p ~/ros2/dock_ws/src && cd ~/ros2/dock_ws/src
git clone https://github.com/Kalman-Robotics/create3_dock_challenge.git
git clone https://github.com/esauarqueros/create3-dock-solution.git create3_dock_solution

# 2. Construir la imagen y levantar el contenedor (el build tarda ~10-15 min la primera vez)
cd create3_dock_solution/docker
export HOST_UID=$(id -u) HOST_GID=$(id -g)
xhost +local:docker
docker compose build
docker compose up -d

# 3. Entrar al contenedor y compilar
docker compose exec dev bash
cd /ws && colcon build --symlink-install
source install/setup.bash     # solo en esta terminal; las nuevas lo cargan solas
```

Cada terminal nueva se abre desde `docker/` con `docker compose exec dev bash`. Dentro del
contenedor, el entorno de ROS y el `install/` del workspace se cargan solos. Para detener el
contenedor: `docker compose down` (desde `docker/`).

Si Gazebo no abre la ventana o se cierra, lanza la simulación con `use_gazebo_gui:=false`.

</details>

<details>
<summary><b>Opción B — ROS 2 Humble nativo (Ubuntu 22.04)</b></summary>

<br>

Requisitos: Ubuntu 22.04 con `ros-humble-desktop`, `python3-colcon-common-extensions` y
`python3-rosdep` (`sudo rosdep init; rosdep update` la primera vez).

```bash
# 1. Workspace y clones
mkdir -p ~/ros2/dock_ws/src && cd ~/ros2/dock_ws/src
git clone https://github.com/Kalman-Robotics/create3_dock_challenge.git
git clone https://github.com/esauarqueros/create3-dock-solution.git create3_dock_solution

# 2. Dependencias: Gazebo Classic, simulación del Create 3, numpy, scipy, matplotlib…
cd ~/ros2/dock_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar
colcon build --symlink-install
source install/setup.bash
```

En cada terminal nueva hay que cargar el entorno:
`source /opt/ros/humble/setup.bash && source ~/ros2/dock_ws/install/setup.bash`.

</details>

## Ejecución

Los pasos son los mismos con Docker o en nativo. Solo cambia cómo abrir cada terminal:
`docker compose exec dev bash` o `source install/setup.bash`.

**0. Limpieza (antes de cada corrida).** Cierra cualquier simulación anterior:

```bash
ros2 run create3_dock_challenge clean_sim.sh
```

Si una compilación anterior quedó a medias o el paquete no aparece, recompila desde cero desde la
raíz del workspace (`/ws` en Docker):

```bash
rm -rf build/ install/ log/
colcon build --symlink-install && source install/setup.bash
```

**1. Terminal 1: el escenario del reto (sin modificar).**

```bash
ros2 launch create3_dock_challenge challenge_world.launch.py
# otra pose inicial:   x:=0.9 y:=0.7 yaw:=-1.2
# sin ventana Gazebo:  use_gazebo_gui:=false
```

Espera a que Gazebo y RViz muestren el robot y el LaserScan.

**2. Terminal 2: la solución.**

*Para el jurado, pedimos correr el siguiente comando, sin ningun parametro adicional, los que tiene por defecto es lo que estamos presentando.*

```bash
ros2 launch create3_dock_solution dock.launch.py
```

En la consola aparecen las transiciones de estado con su tiempo, y una vez por segundo los
errores respecto al dock:

```
[  0.03 s] SEARCH -> GO_TO_PREDOCK (dock detectado)
[  5.12 s] GO_TO_PREDOCK -> ALIGN_AT_PREDOCK (pre-dock a 3.9 cm)
[  6.92 s] ALIGN_AT_PREDOCK -> FINAL_APPROACH (alineado)
[ 13.48 s] FINAL_APPROACH -> DOCKED (is_docked en 13.5 s)
```

**3. Terminal 3 (opcional): comprobar el acople.**

```bash
ros2 topic echo /dock_status --field is_docked
```

**Visualización en RViz (opcional).** Añade un display `MarkerArray` con el tópico
`dock_detector/markers` y `odom` como Fixed Frame:

| Marker | Color |
|---|---|
| Pared del dock detectada | azul claro |
| Pose gruesa de la marca (cajas), antes de filtrar | naranja |
| Pose filtrada de la marca, sobre la pared | verde |
| Dock real: pose objetivo del robot, mirando a la pared | rojo |
| Pre-dock | magenta |

<details>
<summary><b>Opciones del launch</b></summary>

<br>

| Argumento | Default | Para qué |
|---|---|---|
| `use_sim_time` | `true` | `true` en Gazebo; **`false` en el robot real** (allí no hay `/clock`). Si no coincide, el nodo lo avisa a los 3 s |
| `approach_mode` | `point_align` | Cómo llegar al pre-dock: `point_align` (evaluado) o `polar` (experimental) |
| `enable_controller` | `true` | `false`: solo detector y markers, sin publicar `/cmd_vel` (para mover el robot con teleop) |
| `enable_markers` | `true` | `false`: no publica los markers de RViz |
| `run_report` | `false` | `true`: guarda por corrida un CSV, un resumen JSON (tiempos, precisión estimada) y una gráfica |
| `report_dir`, `tag` | `/ws/dock_runs`, `""` | Carpeta y etiqueta del reporte; `scripts/compare_runs.py <carpeta>` compara corridas |

```bash
ros2 launch create3_dock_solution dock.launch.py --show-args      # ver todos los argumentos
ros2 launch create3_dock_solution dock.launch.py use_sim_time:=false   # robot real
```

Todos los parámetros (detector y controller) están en `config/dock_detector.yaml` y
`config/dock_controller.yaml`. Los del controller se pueden cambiar en caliente, p. ej.
`ros2 param set /dock_lidar controller.final.v 0.10`.

</details>

## Algoritmo

Un solo nodo, `dock_node` (nombre ROS `dock_lidar`), con dos piezas en Python puro: percepción y
control. Ninguna depende de ROS, así que se prueban con `pytest`.

**1. Percepción** (`detector.py`, `perception/`), en cada `/scan` (10 Hz):

1. Convierte el scan a puntos (x, y) en `base_link` usando el TF del LiDAR (va montado girado 180°).
   Interpola la pose del robot en el instante del scan y corrige el movimiento durante el barrido.
2. **RANSAC de pared**: ajusta varias paredes candidatas dentro de 3 m.
3. **Validación de la firma**: en cada pared busca dos cajas que sobresalgan ~8 cm, con ~8 cm de
   ancho y un hueco de ~9.5 cm entre ellas. Las tolerancias se relajan con la distancia. Se queda
   con la pared que tiene la firma, no con la más grande.
4. **ICP** a menos de 1 m: alinea un modelo de pared + cajas con los puntos para refinar la
   posición. La orientación se toma de la normal de la pared, que es más precisa.
5. **Filtro temporal en `odom`**: EMA ponderada por confianza, con rechazo de saltos y reinicio
   tras perder la detección.

La salida es la pose de la marca: el centro del hueco sobre la pared, con orientación igual a la
normal hacia la sala.

**2. Objetivos.** El **dock real** (donde debe quedar el centro del robot) está a 26.6 cm de la
pared sobre esa normal, girado 180°: el robot acoplado mira a la pared. El **pre-dock** está
40 cm más atrás sobre el mismo eje, para entrar en línea recta.

**3. Control** (`controller.py`, `control/`), máquina de estados a 20 Hz:

```
SEARCH → GO_TO_PREDOCK → ALIGN_AT_PREDOCK → FINAL_APPROACH → DOCKED
              ↑______________ BACK_OFF ←___________|   (abortos, choques)
```

| Estado | Qué hace |
|---|---|
| SEARCH | Si el detector ya ve el dock, pasa de inmediato. Si no, gira en el sitio y luego hace un arco corto |
| GO_TO_PREDOCK | Tramo rápido (hasta 0.25 m/s): apunta al pre-dock y avanza. La pose del dock se re-estima con cada scan |
| ALIGN_AT_PREDOCK | Gira en el sitio hasta quedar sobre el eje del dock (±2°) |
| FINAL_APPROACH | Tramo lento (8 cm/s, 4 cm/s en los últimos 8 cm). Ley tipo Stanley: el rumbo de referencia corta el eje con un ángulo `atan(k·e_lat)`, así corrige el error lateral de forma continua mientras avanza. Termina cuando `/dock_status` reporta `is_docked: true` |
| DOCKED | Se detiene y mantiene la posición con odometría |
| BACK_OFF | Si se desvía, se pasa del dock sin acoplar, no progresa o choca: retrocede en línea recta y reintenta (máx. 5 intentos) |

Reacciona a `/hazard_detection`:
- **Choque o ruedas trabadas:** se detiene, retrocede y reintenta. Se ignora en los últimos
  centímetros, donde es el contacto normal con el dock.
- **Desnivel:** se detiene.
- **Objeto cercano:** baja la velocidad.

**No hay nada hardcodeado.** La pose del dock siempre sale del LiDAR. Distancias, velocidades,
ganancias y tolerancias son parámetros de `config/*.yaml`.

**Resultados en Gazebo** (`point_align`, ruido del LiDAR 1 mm, tiempo desde que arranca el nodo):

| Pose inicial (x, y, yaw) | Tiempo hasta `is_docked` | Error real lateral / angular |
|---|---|---|
| (0.41, −0.18, 20.6°), por defecto | 13.5 s | — |
| (0.9, 0.7, −69°) | 15.1 s | 2.8 mm / 0.8° |
| (1.3, 0.9, −90°), de costado y cerca de la pared | 16.6 s | — |
| (0.0, −0.9, 180°), de espaldas | 19.5 s | — |

## Limitaciones conocidas

- **Necesita ver las dos cajas.** Desde un ángulo muy rasante el detector no confirma la firma y
  el robot se queda en SEARCH hasta verla. En las pruebas detectó desde todo el rango de evaluación.
- **Los últimos 12 cm dependen de la odometría.** En ese tramo la pose del dock se congela para
  que el ruido no mueva el objetivo. La deriva en esa distancia es despreciable en simulación.
- **Rebote al acoplar.** El cuerpo del dock está ~1 cm más allá del umbral de `is_docked`. A veces
  el robot lo toca, rebota y `is_docked` cae un instante. El controller vuelve a acoplar solo,
  pero el primer `is_docked: true` ya cuenta.
- **Afinado en simulación.** En el robot real hay que lanzar con `use_sim_time:=false`, y puede
  requerir ajustar velocidades por el mayor ruido del LiDAR real.
- **Avisos no verificados en Gazebo.** Las reacciones a objeto cercano y a desnivel de
  `/hazard_detection` no se observaron en simulación.

## Estructura del repo

```
create3_dock_solution/
├── README.md, LICENSE, package.xml, setup.py, setup.cfg, resource/
├── create3_dock_solution/
│   ├── dock_node.py        # único nodo ROS 2: suscripciones, TF, timer 20 Hz, /cmd_vel, markers
│   ├── detector.py         # DockDetector: scan → pose de la marca en odom
│   ├── perception/         # RANSAC, selección de pared, validación de cajas, ICP, filtro, geometría
│   ├── controller.py       # DockController: máquina de estados → Command(v, w)
│   ├── control/            # parámetros, dock real / pre-dock, leyes de control
│   ├── diagnostics/        # reporte por corrida (run_report)
│   └── types.py            # Pose2D, DockEstimate, Command, HazardState
├── config/                 # dock_detector.yaml, dock_controller.yaml
├── launch/dock.launch.py   # comando único
├── scripts/compare_runs.py # compara reportes de corridas
├── test/                   # pytest: percepción, controller, simulador + benchmark offline
└── docker/                 # Dockerfile, docker-compose.yml, ros_env.sh
```

## Tests

Desde la raíz del paquete:

```bash
PYTHONPATH=. python3 -m pytest test/ -q                 # tests unitarios y de integración, sin ROS ni Gazebo
PYTHONPATH=. python3 test/benchmark_controller.py       # controller en 120 poses del rango (uniciclo ideal)
```

<details>
<summary><b>Para el equipo: cómo contribuir</b></summary>

<br>

1. Parte siempre de `main` actualizado: `git switch main && git pull`.
2. Crea una rama por tarea: `git switch -c feat/<tarea>`.
3. Haz commit y push de la rama: `git push -u origin feat/<tarea>`.
4. Abre un Pull Request hacia `main` y pide revisión.

No subir `build/`, `install/`, `log/` ni `dock_runs/`. No hardcodear poses ni coordenadas del
mundo: descalifica.

</details>

## Licencia

Apache-2.0. Ver [`LICENSE`](LICENSE).
