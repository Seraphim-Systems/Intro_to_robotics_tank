from dataclasses import dataclass, field


@dataclass
class LineFollowConfig:
    """Black-line follow tuning parameters (initial placeholders)."""

    base_speed: int = 1300
    turn_speed_delta: int = 900
    lost_line_timeout_s: float = 0.8


@dataclass
class BallDetectConfig:
    """Ball detection parameters for setup phase."""

    # Camera/color segmentation placeholders for dark object tracking
    min_contour_area_px: int = 500
    approach_stop_distance_cm: float = 10.0
    valid_distance_min_cm: float = 3.0
    valid_distance_max_cm: float = 80.0


@dataclass
class ZoneConfig:
    """Drop zone behavior placeholders."""

    approach_speed: int = 900
    release_wait_s: float = 0.6
    retreat_wait_s: float = 0.6


@dataclass
class MissionConfig:
    """Top-level mission configuration."""

    line: LineFollowConfig = field(default_factory=LineFollowConfig)
    ball: BallDetectConfig = field(default_factory=BallDetectConfig)
    zone: ZoneConfig = field(default_factory=ZoneConfig)
    cycle_sleep_s: float = 0.05
