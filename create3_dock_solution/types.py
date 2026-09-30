from dataclasses import dataclass
from typing import Optional

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

@dataclass
class DockEstimate:
    pose: Pose2D        # pose del dock en el frame odom
    confidence: float   # 0..1
    stamp: float        # segundos

@dataclass
class Command:
    v: float            # m/s
    w: float            # rad/s