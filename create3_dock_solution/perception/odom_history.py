"""
Historial corto de odometría para consultar la pose del robot en un instante dado.

El scan y la odometría llegan a destiempo: componer la detección (en
`base_link` en el instante del scan) con el último `/odom` recibido introduce
un error que crece con la velocidad de giro (ver docs/ESTRATEGIA.md §10).
Aquí se interpola la pose en el `stamp` del scan.
"""
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass

from .geometry import integrate_unicycle, wrap_angle
from ..types import Pose2D


@dataclass
class OdomSample:
    stamp: float
    pose: Pose2D
    v: float    # velocidad lineal en base_link (m/s)
    w: float    # velocidad angular (rad/s)


class OdomHistory:
    def __init__(self, max_age_s: float = 1.0, max_extrapolation_s: float = 0.1):
        self.max_age_s = max_age_s
        self.max_extrapolation_s = max_extrapolation_s
        self._samples: deque[OdomSample] = deque()

    def add(self, stamp: float, pose: Pose2D, v: float, w: float):
        if self._samples and stamp <= self._samples[-1].stamp:
            return   # fuera de orden o duplicada
        self._samples.append(OdomSample(stamp, Pose2D(pose.x, pose.y, pose.yaw), v, w))
        while self._samples[0].stamp < stamp - self.max_age_s:
            self._samples.popleft()

    def latest(self) -> OdomSample | None:
        return self._samples[-1] if self._samples else None

    def pose_at(self, stamp: float) -> Pose2D | None:
        """
        Pose en `stamp`: interpolada dentro del historial.

        Si `stamp` es posterior a la última muestra, se extrapola con su (v, w)
        hasta `max_extrapolation_s`. Devuelve `None` si queda fuera de eso.
        """
        if not self._samples:
            return None
        last = self._samples[-1]
        if stamp >= last.stamp:
            dt = stamp - last.stamp
            if dt > self.max_extrapolation_s:
                return None
            return integrate_unicycle(last.pose, last.v, last.w, dt)
        if stamp < self._samples[0].stamp:
            return None

        stamps = [s.stamp for s in self._samples]
        i = bisect_right(stamps, stamp) - 1
        a, b = self._samples[i], self._samples[i + 1]
        k = (stamp - a.stamp) / (b.stamp - a.stamp)
        return Pose2D(
            x=a.pose.x + k * (b.pose.x - a.pose.x),
            y=a.pose.y + k * (b.pose.y - a.pose.y),
            yaw=wrap_angle(a.pose.yaw + k * wrap_angle(b.pose.yaw - a.pose.yaw)),
        )

    def twist_at(self, stamp: float) -> tuple[float, float] | None:
        """(v, w) de la muestra más cercana a `stamp`."""
        if not self._samples:
            return None
        nearest = min(self._samples, key=lambda s: abs(s.stamp - stamp))
        return nearest.v, nearest.w
