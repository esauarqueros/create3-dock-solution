from create3_dock_solution.detector import DockDetector
from create3_dock_solution.types import Pose2D
import numpy as np


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
