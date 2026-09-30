from create3_dock_solution.detector import DockDetector
from create3_dock_solution.perception.geometry import integrate_unicycle
from create3_dock_solution.types import Pose2D
import numpy as np
import pytest


def _synthetic_ranges(
        wall_x=1.2, n=720, angle_min=-np.pi, angle_inc=2 * np.pi / 720,
        box_gap=0.095, box_size=0.08, box_protrusion=0.08, max_range=8.0,
        noise=0.001, seed=0, visible_boxes=('left', 'right')):
    """
    Genera un LaserScan sintético de un robot en base_link viendo la pared del dock.

    Robot en el origen mirando +x, pared en x=wall_x con las cajas de la geometría
    del dock sobresaliendo de ella. `visible_boxes` controla si se renderizan una o
    las dos cajas -- con una sola (p.ej. porque la otra queda oculta por el ángulo
    rasante de vista, como cuando el robot arranca muy a un costado del dock) el
    rayo simplemente sigue hasta la pared llana detrás, tal como lo vería el LIDAR
    real en ese caso.
    """
    rng = np.random.default_rng(seed)
    angles = angle_min + np.arange(n) * angle_inc
    ranges = np.full(n, max_range)
    half_gap = box_gap / 2.0

    for i, a in enumerate(angles):
        if abs(a) > np.deg2rad(60):
            continue
        d = np.cos(a)
        if abs(d) < 1e-6:
            continue
        y_at_wall = wall_x * np.tan(a)
        side = 'left' if y_at_wall > 0 else 'right'
        on_box = half_gap < abs(y_at_wall) < half_gap + box_size and side in visible_boxes
        r = (wall_x - box_protrusion) / d if on_box else wall_x / d
        ranges[i] = max(0.05, r + rng.normal(0.0, noise))

    return ranges, angle_min, angle_inc


# Sala tipo la del reto, solo para generar scans de prueba (el paquete no usa
# coordenadas del mundo): paredes en x=-3.95, x=1.95, y=±1.95; dock en (1.95, 0)
# mirando hacia -x, con las dos cajas sobresaliendo 8 cm.
_ROOM_DOCK = Pose2D(1.95, 0.0, np.pi)


def _room_segments(box_gap=0.095, box_size=0.08, box_protrusion=0.08):
    x_wall, half = 1.95, 1.95
    segs = [((-3.95, -half), (x_wall, -half)), ((x_wall, -half), (x_wall, half)),
            ((x_wall, half), (-3.95, half)), ((-3.95, half), (-3.95, -half))]
    x_front = x_wall - box_protrusion
    for sign in (1.0, -1.0):
        y_in = sign * box_gap / 2.0
        y_out = sign * (box_gap / 2.0 + box_size)
        segs += [((x_front, y_in), (x_front, y_out)),
                 ((x_front, y_in), (x_wall, y_in)), ((x_front, y_out), (x_wall, y_out))]
    return np.array(segs, dtype=float)


def _room_ranges(poses, angle_min=-np.pi, n=720, noise=0.001, seed=0):
    """
    Raycast de la sala: el haz i sale desde `poses[i]` (o `poses` si es una sola pose).

    Pasar una pose por haz permite simular la distorsión de un robot moviéndose
    durante el barrido.
    """
    rng = np.random.default_rng(seed)
    if isinstance(poses, Pose2D):
        poses = [poses] * n
    angle_inc = 2 * np.pi / n
    segs = _room_segments()
    a, e = segs[:, 0, :], segs[:, 1, :] - segs[:, 0, :]
    ranges = np.full(n, np.inf)
    for i, pose in enumerate(poses):
        o = np.array([pose.x, pose.y])
        ang = pose.yaw + angle_min + i * angle_inc
        d = np.array([np.cos(ang), np.sin(ang)])
        # Intersección rayo o + t·d con segmento a + u·e (regla de Cramer).
        denom = d[1] * e[:, 0] - d[0] * e[:, 1]
        ok = np.abs(denom) > 1e-12
        safe = np.where(ok, denom, 1.0)
        rel = a - o
        t = np.where(ok, (rel[:, 1] * e[:, 0] - rel[:, 0] * e[:, 1]) / safe, -1.0)
        u = np.where(ok, (d[0] * rel[:, 1] - d[1] * rel[:, 0]) / safe, -1.0)
        hits = t[(t > 0) & (u >= 0) & (u <= 1)]
        ranges[i] = hits.min() + rng.normal(0.0, noise)
    return ranges, angle_min, angle_inc


def _lateral_error(estimate) -> float:
    """Error del dock estimado a lo largo de la pared (m)."""
    return abs(estimate.pose.y - _ROOM_DOCK.y)


@pytest.mark.parametrize('start', [(0.0, 0.0), (0.41, -0.18)])
def test_detector_detects_dock_from_room_center_with_three_walls_visible(start):
    """
    Desde el centro de la sala hay 3 paredes dentro del rango de búsqueda con ~33%.

    de los puntos cada una: el umbral de ratio de inliers no debe impedir la
    detección (antes, con 0.35, fallaba en todas las orientaciones).
    """
    for yaw in np.linspace(-3.0, 3.0, 7):
        robot = Pose2D(start[0], start[1], float(yaw))
        ranges, angle_min, angle_inc = _room_ranges(robot)
        estimate = DockDetector(params={}).update(ranges, angle_min, angle_inc, robot, 0.0)

        assert estimate is not None, f'sin detección desde {start}, yaw={yaw:.2f}'
        assert abs(estimate.pose.x - _ROOM_DOCK.x) < 0.01
        assert _lateral_error(estimate) < 0.01
        yaw_err = abs(np.angle(np.exp(1j * (estimate.pose.yaw - _ROOM_DOCK.yaw))))
        assert yaw_err < np.deg2rad(1.0)


def test_detector_deskew_compensates_rotation_during_sweep():
    """Robot girando a 1 rad/s: el barrido de 0.1 s desplaza el dock si no se corrige."""
    v, w = 0.0, 1.0
    n = 720
    time_increment = 0.1 / n
    start = Pose2D(0.5, 0.2, 0.3)
    poses = [integrate_unicycle(start, v, w, i * time_increment) for i in range(n)]
    ranges, angle_min, angle_inc = _room_ranges(poses)

    def detect(deskew: bool):
        detector = DockDetector(params={'deskew': {'enable': deskew}})
        return detector.update(ranges, angle_min, angle_inc, start, 0.0,
                               time_increment=time_increment, twist=(v, w))

    corrected, raw = detect(True), detect(False)

    assert corrected is not None
    assert _lateral_error(corrected) < 0.01
    assert raw is None or _lateral_error(raw) > 3 * _lateral_error(corrected)


def test_detector_detects_dock_when_facing_it():
    ranges, angle_min, angle_inc = _synthetic_ranges(wall_x=1.2)
    detector = DockDetector(params={})
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    estimate = None
    for i in range(5):
        estimate = detector.update(ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)

    assert estimate is not None
    assert estimate.confidence > 0.3
    assert abs(estimate.pose.x - 1.2) < 0.05
    assert abs(estimate.pose.y) < 0.05


def test_detector_returns_none_or_low_confidence_from_the_side():
    """
    Robot a un costado del dock: por el ángulo rasante solo se ve una caja.

    Sin la geometría completa (hueco + ambas cajas) no hay detección confiable.
    Este es el caso que le interesa a la recuperación del controller (ir al
    centro de la sala para mejorar el ángulo, ver docs/ESTRATEGIA.md).
    """
    ranges, angle_min, angle_inc = _synthetic_ranges(wall_x=1.2, visible_boxes=('left',))
    detector = DockDetector(params={})
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    estimate = None
    for i in range(5):
        estimate = detector.update(ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)

    assert estimate is None or estimate.confidence < 0.3


def test_detector_detects_dock_at_longer_range_with_graceful_confidence():
    """
    Reproduce el Escenario 2 (docs/ESTRATEGIA.md 9.2).

    Con la pared correcta ya identificada sin ambigüedad (robot centrado), el
    detector no debe necesitar acercarse a <1.4m para dar una estimación, con
    una confianza usable (no cercana a 0) en todo el rango. No se exige que la
    confianza decrezca monótonamente punto a punto: con pocas muestras por
    caja, cuántas caen exactamente sobre ella depende de la fase de alineación
    entre la rejilla angular fija del LIDAR (0.5°) y la ventana angular de la
    caja en cada distancia puntual -- un artefacto de cuantización esperable
    con tan pocos puntos, no un defecto del detector (la tendencia real, sobre
    muchas orientaciones/posiciones, sí es decreciente; ver diagnóstico en
    docs/ESTRATEGIA.md 9.2). Se evita el límite exacto de `wall_search_range_max`
    (3.0m) porque ahí el filtro por rango euclidiano deja fuera casi toda la
    pared salvo el punto exactamente al frente (geometría del campo de visión,
    no relacionado con la validación de cajas).
    """
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    for wall_x in (1.5, 2.0, 2.5, 2.9):
        ranges, angle_min, angle_inc = _synthetic_ranges(wall_x=wall_x)
        detector = DockDetector(params={})
        estimate = None
        for i in range(8):
            estimate = detector.update(
                ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)
        assert estimate is not None, f'sin detección a wall_x={wall_x}'
        assert estimate.confidence > 0.5, f'confidence demasiado baja a wall_x={wall_x}'
        assert abs(estimate.pose.x - wall_x) < 0.05
        assert abs(estimate.pose.y) < 0.05


def test_detector_confidence_trend_decreases_with_distance_on_average():
    """
    Confirma la tendencia real (promediada sobre varias fases angulares).

    Una sola distancia puntual puede verse afectada por en qué fase cae la
    rejilla angular fija respecto a la ventana de la caja (ver test anterior).
    Promediando sobre varios corrimientos angulares pequeños, la tendencia de
    fondo sí es decreciente con la distancia, como predice el modelo de
    cobertura de `coarse_confidence` (docs/ESTRATEGIA.md 9.2).
    """
    robot_pose = Pose2D(0.0, 0.0, 0.0)
    angle_offsets = np.linspace(0.0, np.deg2rad(0.4), 6)   # sub-muestrea la fase de la rejilla

    def mean_confidence(wall_x: float) -> float:
        confidences = []
        for offset in angle_offsets:
            ranges, angle_min, angle_inc = _synthetic_ranges(
                wall_x=wall_x, angle_min=-np.pi + offset)
            detector = DockDetector(params={})
            estimate = None
            for i in range(3):
                estimate = detector.update(
                    ranges, angle_min, angle_inc, robot_pose, stamp=float(i) * 0.1)
            if estimate is not None:
                confidences.append(estimate.confidence)
        assert confidences, f'ninguna fase detectó el dock a wall_x={wall_x}'
        return float(np.mean(confidences))

    assert mean_confidence(1.5) > mean_confidence(2.9)
