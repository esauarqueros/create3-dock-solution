from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class DockEstimate:
    pose: Pose2D        # pose del marcador (centro del hueco en la pared) en el frame odom
    confidence: float   # 0..1
    stamp: float        # segundos


@dataclass
class Command:
    v: float            # m/s
    w: float            # rad/s


@dataclass
class HazardState:
    """Resumen de /hazard_detection (ver docs/CONTROLADOR_IMPLEMENTADO.md)."""

    bump: bool = False       # BUMP: contacto del parachoques
    stall: bool = False      # STALL: ruedas trabadas
    cliff: bool = False      # CLIFF o WHEEL_DROP
    proximity: bool = False  # OBJECT_PROXIMITY
