"""
Filtro temporal de la pose del dock en frame `odom`.

EMA con ganancia adaptativa por confidence (ver docs/ESTRATEGIA.md, filtrado
obligatorio). Devuelve `None` cuando no hay una pose confiable -- esta es la
señal explícita que debe consumir `controller.py` (p.ej. para decidir ir al
centro de la sala cuando el robot arranca a un costado del dock y no ve
ambas cajas).
"""
import numpy as np

from .dock_model import FilterParams
from .geometry import wrap_angle
from ..types import DockEstimate, Pose2D


class DockPoseFilter:
    def __init__(self, params: FilterParams):
        self.p = params
        self._pose: Pose2D | None = None
        self._confidence = 0.0
        self._time_since_valid = float('inf')
        self._last_stamp: float | None = None
        # Racha de mediciones rechazadas por salto pero coherentes entre sí.
        self._rejected_last: Pose2D | None = None
        self._rejected_count = 0

    def update(self, measurement: Pose2D | None, confidence: float,
               stamp: float) -> DockEstimate | None:
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp

        accept = measurement is not None and confidence > 0.0
        # Sin pose, o perdida más de lost_timeout_s: la pose guardada ya no es
        # una referencia válida, así que la siguiente medición la reemplaza en
        # vez de mezclarse por EMA con ella.
        reinit = accept and (self._pose is None or self._is_lost())

        if accept and not reinit and self._confidence > self.p.outlier_reject_confidence:
            jump = float(np.hypot(measurement.x - self._pose.x, measurement.y - self._pose.y))
            if jump > self.p.max_jump_m:
                accept = False
                # Si el filtro se enganchó a una detección errónea, las correctas
                # llegan todas "lejos" pero coherentes entre sí: tras varias
                # seguidas se re-engancha a ellas.
                reinit = self._track_rejected(measurement)

        if reinit:
            self._pose = Pose2D(measurement.x, measurement.y, measurement.yaw)
            self._confidence = float(np.clip(confidence, 0.0, 1.0))
            self._time_since_valid = 0.0
            self._reset_rejected()
        elif accept:
            gain = self.p.alpha_min + confidence * (self.p.alpha_max - self.p.alpha_min)
            k = float(np.clip(gain, self.p.alpha_min, self.p.alpha_max))
            self._pose.x += k * (measurement.x - self._pose.x)
            self._pose.y += k * (measurement.y - self._pose.y)
            yaw_delta = k * wrap_angle(measurement.yaw - self._pose.yaw)
            self._pose.yaw = wrap_angle(self._pose.yaw + yaw_delta)
            new_confidence = self._confidence + k * (confidence - self._confidence)
            self._confidence = float(np.clip(new_confidence, 0.0, 1.0))
            self._time_since_valid = 0.0
            self._reset_rejected()
        else:
            self._confidence *= self.p.confidence_decay_per_s ** dt
            self._time_since_valid += dt

        if self._pose is None or self._is_lost():
            return None
        confidence_out = float(np.clip(self._confidence, 0.0, 1.0))
        pose_out = Pose2D(self._pose.x, self._pose.y, self._pose.yaw)
        return DockEstimate(pose=pose_out, confidence=confidence_out, stamp=stamp)

    def _is_lost(self) -> bool:
        return self._time_since_valid > self.p.lost_timeout_s

    def _track_rejected(self, measurement: Pose2D) -> bool:
        last = self._rejected_last
        coherent = last is not None and float(np.hypot(
            measurement.x - last.x, measurement.y - last.y)) <= self.p.reinit_consistency_m
        self._rejected_count = self._rejected_count + 1 if coherent else 1
        self._rejected_last = measurement
        return self._rejected_count >= self.p.reinit_after_consecutive

    def _reset_rejected(self):
        self._rejected_last = None
        self._rejected_count = 0
