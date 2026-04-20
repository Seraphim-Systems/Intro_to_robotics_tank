from dataclasses import dataclass, field


@dataclass
class LineFollowConfig:
    """Black-line follow tuning parameters (initial placeholders)."""

    base_speed: int = 1300
    turn_speed_delta: int = 250
    lost_line_timeout_s: float = 0.8
    code_zero_is_center: bool = True
    use_car_mode_infrared: bool = True
    lost_line_crawl_speed: int = 260


@dataclass
class ObstacleAvoidConfig:
    """Ultrasonic obstacle avoidance tuning for autonomous mode."""

    trigger_distance_cm: float = 15.0
    backup_speed: int = -1300
    backup_duration_s: float = 0.35
    turn_left_speed: int = -1100
    turn_right_speed: int = 1100
    turn_duration_s: float = 0.35
    bypass_speed: int = 900
    bypass_duration_s: float = 0.45
    return_turn_duration_s: float = 0.30
    settle_duration_s: float = 0.15


@dataclass
class BallDetectConfig:
    """Ball detection parameters for setup phase."""

    # Red ball HSV thresholds use two windows to span hue wrap-around.
    hsv_lower_1: tuple[int, int, int] = (0, 118, 31)
    hsv_upper_1: tuple[int, int, int] = (6, 255, 255)
    hsv_lower_2: tuple[int, int, int] = (170, 118, 31)
    hsv_upper_2: tuple[int, int, int] = (179, 255, 255)
    min_contour_area_px: int = 500
    min_radius_px: float = 12.0
    center_tolerance_px: int = 30
    approach_speed: int = 520
    steer_speed_delta: int = 220
    search_spin_speed: int = 500
    lost_target_timeout_s: float = 1.0
    approach_stop_distance_cm: float = 15.0
    approach_slowdown_distance_cm: float = 40.0
    approach_slow_speed: int = 420
    approach_final_distance_cm: float = 24.0
    approach_final_speed: int = 300
    valid_distance_min_cm: float = 3.0
    valid_distance_max_cm: float = 80.0
    invert_steering: bool = True
    pre_open_clamp_before_pick: bool = True


@dataclass
class HomeMarkerConfig:
    """Runtime-calibrated drop-zone marker behavior and thresholds."""

    calibration_roi_px: int = 70
    calibration_samples: int = 14
    hue_tolerance: int = 10
    sat_tolerance: int = 60
    val_tolerance: int = 60
    min_contour_area_px: int = 700
    min_radius_px: float = 14.0
    center_tolerance_px: int = 35
    approach_speed: int = 850
    steer_speed_delta: int = 420
    search_spin_speed: int = 650
    lost_target_timeout_s: float = 1.5
    drop_distance_cm: float = 10.0
    valid_distance_min_cm: float = 3.0
    valid_distance_max_cm: float = 100.0
    invert_steering: bool = True


@dataclass
class ZoneConfig:
    """Drop zone behavior placeholders."""

    approach_speed: int = 900
    release_wait_s: float = 0.6
    retreat_wait_s: float = 0.6


@dataclass
class GeoNavConfig:
    """Dead-reckoning geolocation and lightweight map controls."""

    duty_to_mps: float = 0.00022
    wheel_base_m: float = 0.16
    home_arrival_radius_m: float = 0.22
    heading_tolerance_rad: float = 0.28
    obstacle_block_radius_m: float = 0.25
    obstacle_memory_s: float = 20.0
    line_sample_stride_m: float = 0.05
    max_map_points: int = 300


@dataclass
class MissionConfig:
    """Top-level mission configuration."""

    line: LineFollowConfig = field(default_factory=LineFollowConfig)
    obstacle: ObstacleAvoidConfig = field(default_factory=ObstacleAvoidConfig)
    ball: BallDetectConfig = field(default_factory=BallDetectConfig)
    home: HomeMarkerConfig = field(default_factory=HomeMarkerConfig)
    zone: ZoneConfig = field(default_factory=ZoneConfig)
    geo: GeoNavConfig = field(default_factory=GeoNavConfig)
    cycle_sleep_s: float = 0.05
