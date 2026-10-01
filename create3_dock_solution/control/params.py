"""
Dataclasses de parámetros del controller.

Cada dataclass mapea 1:1 a un grupo `controller.<grupo>` de
`config/dock_controller.yaml`. Los valores por defecto son los mismos que ese
archivo; `dock_node` los declara como parámetros ROS a partir de
`ControllerParams().to_nested_dict()`, así que no hay una tercera copia a mano.
Ángulos en grados en el YAML (más legibles al afinar); se convierten donde se usan.
"""
from dataclasses import asdict, dataclass, field, fields

APPROACH_MODES = ('point_align', 'polar')


def _from_dict(cls, data: dict):
    data = data or {}
    known = {f.name for f in fields(cls)}
    return cls(**{k: v for k, v in data.items() if k in known})


@dataclass
class SupervisionParams:
    enable: bool = True                 # false: no comanda (modo solo detector / teleop)
    dock_max_age_s: float = 0.5         # scan más viejo que esto -> se trata como sin detección
    min_confidence: float = 0.3         # confianza mínima para salir de SEARCH
    max_attempts: int = 5               # intentos abortados antes de FAILED
    approach_timeout_s: float = 60.0
    align_timeout_s: float = 15.0
    final_timeout_s: float = 40.0
    progress_eps_m: float = 0.01        # mejora mínima de e_long en FINAL...
    progress_timeout_s: float = 6.0     # ...en esta ventana, si no -> abortar intento
    undock_grace_s: float = 1.0         # is_docked cae a false más que esto en DOCKED -> FINAL
    # Mantener la posición ya acoplado (control P con odometría sobre el eje).
    # En Gazebo, con v=0 el robot rueda hacia atrás por la rampa, y empujando
    # choca con el cuerpo del dock ~1 cm más adelante y rebota: en ambos casos
    # pierde is_docked. docked_hold_v = 0 desactiva (solo parar).
    docked_hold_v: float = 0.02         # |v| máxima de la corrección
    docked_hold_k: float = 1.5          # 1/s
    docked_hold_advance_m: float = 0.003  # objetivo: un poco más adentro que donde acopló

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class GeometryParams:
    # Pared -> centro del robot acoplado, sobre la normal (README del reto: ~26.6 cm).
    dock_offset_m: float = 0.266
    # Dock real -> pre-dock, sobre el eje hacia la sala (0.40 m => ~0.67 m de la pared).
    predock_distance_m: float = 0.40

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class SearchParams:
    wait_s: float = 0.5         # quieto: el filtro necesita 1-3 scans
    w: float = 0.5              # giro en sitio (lento: no degradar el scan)
    arc_v: float = 0.10         # tras una vuelta completa sin detección: arco corto
    arc_w: float = 0.4
    arc_time_s: float = 3.0

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class ApproachParams:
    mode: str = 'point_align'   # 'point_align' (B1 al pre-dock + ALIGN) | 'polar' (B3)
    v_max: float = 0.25
    v_min: float = 0.04
    k_v: float = 0.8
    w_max: float = 1.0
    k_alpha: float = 1.5
    rotate_in_place_deg: float = 60.0
    tolerance_m: float = 0.04   # point_align: llegada al pre-dock

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class PolarParams:
    # Astolfi: estable con k_rho > 0, k_beta < 0, k_alpha > k_rho.
    k_rho: float = 0.5
    k_alpha: float = 1.5
    k_beta: float = -0.6
    v_min_pass: float = 0.08    # no frenar a 0 en el pre-dock: se pasa de largo a FINAL
    switch_dist_m: float = 0.08
    switch_yaw_deg: float = 10.0
    switch_lat_m: float = 0.03

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class AlignParams:
    k_theta: float = 1.0
    w_max: float = 0.6
    w_min: float = 0.08
    tolerance_deg: float = 2.0
    abort_lat_m: float = 0.08

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class FinalParams:
    v: float = 0.08
    v_slow: float = 0.04
    slow_zone_m: float = 0.08       # |e_long| menor que esto -> v_slow
    k_lat: float = 5.0
    max_lat_offset_deg: float = 20.0
    k_yaw: float = 1.5
    w_max: float = 0.5
    abort_lat_m: float = 0.08
    abort_yaw_deg: float = 25.0
    overshoot_m: float = 0.03       # pasar el dock esto sin is_docked -> reintento
    freeze_within_m: float = 0.12   # congelar el objetivo en los últimos cm

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class BackoffParams:
    distance_m: float = 0.25
    v: float = 0.12
    timeout_s: float = 5.0

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class HazardParams:
    hold_s: float = 0.3                 # un hazard sigue activo este tiempo tras el último mensaje
    # En FINAL, un bump a menos de esto del dock es el contacto esperado con el dock.
    bump_ignore_within_m: float = 0.06
    proximity_speed_scale: float = 0.5  # OBJECT_PROXIMITY en SEARCH/APPROACH -> v *= esto

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


@dataclass
class LimitsParams:
    v_abs_max: float = 0.30     # Create 3: ~0.306 m/s
    w_abs_max: float = 1.9
    max_lin_acc: float = 0.5    # m/s^2, 0 = sin rampa
    max_ang_acc: float = 3.0    # rad/s^2, 0 = sin rampa

    @classmethod
    def from_dict(cls, data: dict):
        return _from_dict(cls, data)


_GROUPS = {
    'supervision': SupervisionParams, 'geometry': GeometryParams, 'search': SearchParams,
    'approach': ApproachParams, 'polar': PolarParams, 'align': AlignParams,
    'final': FinalParams, 'backoff': BackoffParams, 'hazard': HazardParams,
    'limits': LimitsParams,
}


@dataclass
class ControllerParams:
    supervision: SupervisionParams = field(default_factory=SupervisionParams)
    geometry: GeometryParams = field(default_factory=GeometryParams)
    search: SearchParams = field(default_factory=SearchParams)
    approach: ApproachParams = field(default_factory=ApproachParams)
    polar: PolarParams = field(default_factory=PolarParams)
    align: AlignParams = field(default_factory=AlignParams)
    final: FinalParams = field(default_factory=FinalParams)
    backoff: BackoffParams = field(default_factory=BackoffParams)
    hazard: HazardParams = field(default_factory=HazardParams)
    limits: LimitsParams = field(default_factory=LimitsParams)

    @classmethod
    def from_dict(cls, data: dict):
        data = data or {}
        return cls(**{name: group.from_dict(data.get(name, {}))
                      for name, group in _GROUPS.items()})

    def to_nested_dict(self) -> dict:
        return asdict(self)

    def validate(self) -> list[str]:
        """Errores de configuración (lista vacía si todo es coherente)."""
        errors = []
        if self.approach.mode not in APPROACH_MODES:
            errors.append(f'approach.mode debe ser uno de {APPROACH_MODES}')
        pol = self.polar
        if pol.k_rho <= 0 or pol.k_beta >= 0 or pol.k_alpha <= pol.k_rho:
            errors.append('polar: se requiere k_rho > 0, k_beta < 0 y k_alpha > k_rho')
        if self.geometry.predock_distance_m <= 0 or self.geometry.dock_offset_m <= 0:
            errors.append('geometry: distancias deben ser > 0')
        for name, value in (('approach.v_max', self.approach.v_max), ('final.v', self.final.v),
                            ('final.v_slow', self.final.v_slow), ('backoff.v', self.backoff.v)):
            if value < 0 or value > self.limits.v_abs_max:
                errors.append(f'{name} debe estar en [0, limits.v_abs_max]')
        return errors
