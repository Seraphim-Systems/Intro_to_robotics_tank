import enum
import math
import time
from dataclasses import dataclass
from typing import Optional

from .config import MissionConfig
from .interfaces import CarAdapter, SensorSnapshot


class MissionState(enum.Enum):
    FOLLOW_LINE = "follow_line"
    APPROACH_BALL = "approach_ball"
    PICK_BALL = "pick_ball"
    GO_TO_ZONE = "go_to_zone"
    DROP_BALL = "drop_ball"
    RECOVER = "recover"


class AutonomousMission:
    """Mission setup state machine.

    This is setup-only scaffolding; algorithms are intentionally conservative placeholders.
    """

    def __init__(self, adapter: CarAdapter, config: MissionConfig):
        self.adapter = adapter
        self.config = config
        self.state = MissionState.FOLLOW_LINE
        self._last_line_seen_ts = time.time()

    def run_once(self) -> None:
        sensors = self.adapter.read_sensors()

        if self.state == MissionState.FOLLOW_LINE:
            self._handle_follow_line(sensors)
        elif self.state == MissionState.APPROACH_BALL:
            self._handle_approach_ball(sensors)
        elif self.state == MissionState.PICK_BALL:
            self._handle_pick_ball(sensors)
        elif self.state == MissionState.GO_TO_ZONE:
            self._handle_go_to_zone(sensors)
        elif self.state == MissionState.DROP_BALL:
            self._handle_drop_ball(sensors)
        elif self.state == MissionState.RECOVER:
            self._handle_recover(sensors)

    def _handle_follow_line(self, sensors: SensorSnapshot) -> None:
        if sensors.ball_visible:
            self.adapter.stop_motors()
            self.state = MissionState.APPROACH_BALL
            return

        # Existing IR encoding in current codebase:
        # left<<2 | middle<<1 | right
        ir = sensors.infrared_code
        if ir in (2, 5) or (ir == 0 and self.config.line.code_zero_is_center):
            self._last_line_seen_ts = time.time()
            self.adapter.set_wheels(
                self.config.line.base_speed, self.config.line.base_speed
            )
        elif ir in (4, 6):
            self._last_line_seen_ts = time.time()
            self.adapter.set_wheels(
                self.config.line.base_speed - self.config.line.turn_speed_delta,
                self.config.line.base_speed + self.config.line.turn_speed_delta,
            )
        elif ir in (1, 3):
            self._last_line_seen_ts = time.time()
            self.adapter.set_wheels(
                self.config.line.base_speed + self.config.line.turn_speed_delta,
                self.config.line.base_speed - self.config.line.turn_speed_delta,
            )
        else:
            # Line lost handling for setup phase.
            if (
                time.time() - self._last_line_seen_ts
                > self.config.line.lost_line_timeout_s
            ):
                self.state = MissionState.RECOVER
                self.adapter.stop_motors()

    def _handle_approach_ball(self, sensors: SensorSnapshot) -> None:
        d = sensors.distance_cm
        if d < 0:
            self.adapter.stop_motors()
            return

        if (
            self.config.ball.valid_distance_min_cm
            <= d
            <= self.config.ball.approach_stop_distance_cm
        ):
            self.adapter.stop_motors()
            self.state = MissionState.PICK_BALL
        else:
            self.adapter.set_wheels(900, 900)

    def _handle_pick_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self.adapter.clamp_pick()
        self.state = MissionState.GO_TO_ZONE

    def _handle_go_to_zone(self, sensors: SensorSnapshot) -> None:
        # Setup placeholder: re-use line follow until zone detection is implemented.
        _ = sensors
        self.adapter.set_wheels(
            self.config.zone.approach_speed, self.config.zone.approach_speed
        )
        # Replace this transition with real zone detection trigger in next iteration.
        self.state = MissionState.DROP_BALL

    def _handle_drop_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self.adapter.stop_motors()
        time.sleep(self.config.zone.release_wait_s)
        self.adapter.clamp_drop()
        time.sleep(self.config.zone.retreat_wait_s)
        self.state = MissionState.FOLLOW_LINE

    def _handle_recover(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        # Minimal recover strategy for setup.
        self.adapter.set_wheels(-450, 450)
        time.sleep(0.12)
        self.state = MissionState.FOLLOW_LINE


class HomeCycleState(enum.Enum):
    FOLLOW_LINE = "follow_line"
    AVOID_OBSTACLE = "avoid_obstacle"
    APPROACH_BALL = "approach_ball"
    PICK_BALL = "pick_ball"
    RETURN_HOME = "return_home"
    DROP_BALL = "drop_ball"
    RECOVER = "recover"


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


@dataclass
class MapPoint:
    x_m: float
    y_m: float
    timestamp: float
    kind: str


class LineAvoidBallHomeMission:
    """Priority-driven autonomous loop with geolocation home return and mapping."""

    def __init__(self, adapter: CarAdapter, config: MissionConfig):
        self.adapter = adapter
        self.config = config
        self.state = HomeCycleState.FOLLOW_LINE

        self._last_line_seen_ts = time.time()
        self._ball_last_seen_ts = 0.0
        self._carrying_ball = False

        self._avoid_phase = ""
        self._avoid_phase_end_ts = 0.0
        self._avoid_resume_state = HomeCycleState.FOLLOW_LINE

        self.pose = Pose2D()
        self.home_pose = Pose2D()
        self._last_motion_ts = time.monotonic()
        self._cmd_left = 0
        self._cmd_right = 0

        self.line_map: list[MapPoint] = []
        self.obstacle_map: list[MapPoint] = []
        self._last_line_sample_pose = Pose2D()

    def run_once(self) -> None:
        self._integrate_pose()
        sensors = self.adapter.read_sensors()
        self._record_map(sensors)

        if self.state == HomeCycleState.AVOID_OBSTACLE:
            self._handle_avoid_obstacle()
            return

        if self._should_start_avoidance(sensors):
            if self.state in (
                HomeCycleState.FOLLOW_LINE,
                HomeCycleState.RETURN_HOME,
                HomeCycleState.APPROACH_BALL,
            ):
                self._start_avoidance(self.state)
                self._handle_avoid_obstacle()
                return

        if self.state == HomeCycleState.FOLLOW_LINE:
            self._handle_follow_line(sensors)
        elif self.state == HomeCycleState.APPROACH_BALL:
            self._handle_approach_ball(sensors)
        elif self.state == HomeCycleState.PICK_BALL:
            self._handle_pick_ball(sensors)
        elif self.state == HomeCycleState.RETURN_HOME:
            self._handle_return_home(sensors)
        elif self.state == HomeCycleState.DROP_BALL:
            self._handle_drop_ball(sensors)
        elif self.state == HomeCycleState.RECOVER:
            self._handle_recover(sensors)

    def get_navigation_status(self) -> dict[str, float | int]:
        return {
            "x_m": self.pose.x_m,
            "y_m": self.pose.y_m,
            "heading_deg": math.degrees(self.pose.heading_rad),
            "home_distance_m": self._distance_to_home(),
            "line_points": len(self.line_map),
            "obstacles": len(self.obstacle_map),
        }

    def reset_home_anchor(self) -> None:
        self.home_pose = Pose2D(self.pose.x_m, self.pose.y_m, self.pose.heading_rad)

    def _should_start_avoidance(self, sensors: SensorSnapshot) -> bool:
        if self.state == HomeCycleState.APPROACH_BALL and not self._carrying_ball:
            # While actively tracking a pickup target, do not preempt with obstacle avoid.
            if sensors.ball_visible or (
                time.time() - self._ball_last_seen_ts
                <= self.config.ball.lost_target_timeout_s
            ):
                return False

        distance = sensors.distance_cm
        return 0 < distance <= self.config.obstacle.trigger_distance_cm

    def _start_avoidance(self, resume_state: HomeCycleState) -> None:
        self.state = HomeCycleState.AVOID_OBSTACLE
        self._avoid_resume_state = resume_state
        self._avoid_phase = "backup"
        self._avoid_phase_end_ts = time.time() + self.config.obstacle.backup_duration_s

    def _handle_avoid_obstacle(self) -> None:
        now = time.time()
        cfg = self.config.obstacle

        if self._avoid_phase == "backup":
            self._drive(cfg.backup_speed, cfg.backup_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "turn"
                self._avoid_phase_end_ts = now + cfg.turn_duration_s
            return

        if self._avoid_phase == "turn":
            self._drive(cfg.turn_left_speed, cfg.turn_right_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "bypass"
                self._avoid_phase_end_ts = now + cfg.bypass_duration_s
            return

        if self._avoid_phase == "bypass":
            self._drive(cfg.bypass_speed, cfg.bypass_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "return"
                self._avoid_phase_end_ts = now + cfg.return_turn_duration_s
            return

        if self._avoid_phase == "return":
            self._drive(cfg.turn_right_speed, cfg.turn_left_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "settle"
                self._avoid_phase_end_ts = now + cfg.settle_duration_s
            return

        self._stop_drive()
        if now >= self._avoid_phase_end_ts:
            self._avoid_phase = ""
            self.state = self._avoid_resume_state

    def _handle_follow_line(self, sensors: SensorSnapshot) -> None:
        if sensors.ball_visible:
            self._ball_last_seen_ts = time.time()
            self._stop_drive()
            self.state = HomeCycleState.APPROACH_BALL
            return

        if self._apply_line_follow_drive(sensors.infrared_code):
            self._last_line_seen_ts = time.time()
            return

        if time.time() - self._last_line_seen_ts > self.config.line.lost_line_timeout_s:
            self._stop_drive()
            self.state = HomeCycleState.RECOVER

    def _handle_approach_ball(self, sensors: SensorSnapshot) -> None:
        now = time.time()
        ball_cfg = self.config.ball

        if sensors.ball_visible:
            self._ball_last_seen_ts = now
        elif now - self._ball_last_seen_ts > ball_cfg.lost_target_timeout_s:
            self._stop_drive()
            self.state = HomeCycleState.FOLLOW_LINE
            return

        if (
            ball_cfg.valid_distance_min_cm
            <= sensors.distance_cm
            <= ball_cfg.approach_stop_distance_cm
        ):
            self._stop_drive()
            self.state = HomeCycleState.PICK_BALL
            return

        if sensors.ball_visible and sensors.ball_center_x is not None:
            approach_speed = self._ball_approach_speed_for_distance(sensors.distance_cm)
            steer_delta = max(
                120,
                int(
                    ball_cfg.steer_speed_delta
                    * approach_speed
                    / max(1, ball_cfg.approach_speed)
                ),
            )
            self._steer_toward_target(
                center_x=sensors.ball_center_x,
                frame_width=sensors.frame_width_px,
                base_speed=approach_speed,
                steer_delta=steer_delta,
                center_tolerance=ball_cfg.center_tolerance_px,
                invert_steering=ball_cfg.invert_steering,
            )
            return

        self._drive(-ball_cfg.search_spin_speed, ball_cfg.search_spin_speed)

    def _handle_pick_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self._stop_drive()
        self.adapter.clamp_pick()
        self._carrying_ball = True
        self.state = HomeCycleState.RETURN_HOME

    def _handle_return_home(self, sensors: SensorSnapshot) -> None:
        _ = sensors

        if self._path_to_home_blocked():
            self._start_avoidance(HomeCycleState.RETURN_HOME)
            self._handle_avoid_obstacle()
            return

        if self._distance_to_home() <= self.config.geo.home_arrival_radius_m:
            self._stop_drive()
            self.state = HomeCycleState.DROP_BALL
            return

        target_heading = math.atan2(
            self.home_pose.y_m - self.pose.y_m,
            self.home_pose.x_m - self.pose.x_m,
        )
        heading_error = self._normalize_angle(target_heading - self.pose.heading_rad)

        if abs(heading_error) > self.config.geo.heading_tolerance_rad:
            turn_speed = max(300, self.config.home.search_spin_speed)
            if heading_error > 0:
                self._drive(-turn_speed, turn_speed)
            else:
                self._drive(turn_speed, -turn_speed)
            return

        forward_speed = max(450, min(self.config.line.base_speed, 1000))
        self._drive(forward_speed, forward_speed)

    def _handle_drop_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self._stop_drive()
        self.adapter.clamp_drop()
        self._carrying_ball = False
        self._last_line_seen_ts = time.time()
        self.state = HomeCycleState.FOLLOW_LINE

    def _handle_recover(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self._drive(-450, 450)
        time.sleep(0.12)
        self._stop_drive()
        self._last_line_seen_ts = time.time()
        if self._carrying_ball:
            self.state = HomeCycleState.RETURN_HOME
        else:
            self.state = HomeCycleState.FOLLOW_LINE

    def _apply_line_follow_drive(self, infrared_code: int) -> bool:
        if infrared_code in (2, 5) or (
            infrared_code == 0 and self.config.line.code_zero_is_center
        ):
            self._drive(
                self.config.line.base_speed,
                self.config.line.base_speed,
            )
            return True

        if infrared_code in (4, 6):
            self._drive(
                self.config.line.base_speed - self.config.line.turn_speed_delta,
                self.config.line.base_speed + self.config.line.turn_speed_delta,
            )
            return True

        if infrared_code in (1, 3):
            self._drive(
                self.config.line.base_speed + self.config.line.turn_speed_delta,
                self.config.line.base_speed - self.config.line.turn_speed_delta,
            )
            return True

        return False

    def _steer_toward_target(
        self,
        center_x: float,
        frame_width: Optional[int],
        base_speed: int,
        steer_delta: int,
        center_tolerance: int,
        invert_steering: bool,
    ) -> None:
        if frame_width is None or frame_width <= 0:
            self._drive(base_speed, base_speed)
            return

        mid_x = frame_width / 2.0
        error = center_x - mid_x
        if invert_steering:
            error = -error

        if abs(error) <= center_tolerance:
            self._drive(base_speed, base_speed)
        elif error < 0:
            self._drive(base_speed - steer_delta, base_speed + steer_delta)
        else:
            self._drive(base_speed + steer_delta, base_speed - steer_delta)

    def _ball_approach_speed_for_distance(self, distance_cm: float) -> int:
        ball_cfg = self.config.ball
        if distance_cm <= 0:
            return ball_cfg.approach_slow_speed
        if distance_cm <= ball_cfg.approach_final_distance_cm:
            return ball_cfg.approach_final_speed
        if distance_cm <= ball_cfg.approach_slowdown_distance_cm:
            return ball_cfg.approach_slow_speed
        return ball_cfg.approach_speed

    def _record_map(self, sensors: SensorSnapshot) -> None:
        now = time.time()
        geo = self.config.geo

        self.obstacle_map = [
            point
            for point in self.obstacle_map
            if now - point.timestamp <= geo.obstacle_memory_s
        ]

        if self._line_detected(sensors.infrared_code):
            dist = self._distance_between(
                self.pose.x_m,
                self.pose.y_m,
                self._last_line_sample_pose.x_m,
                self._last_line_sample_pose.y_m,
            )
            if dist >= geo.line_sample_stride_m:
                self.line_map.append(
                    MapPoint(self.pose.x_m, self.pose.y_m, now, "line")
                )
                self._last_line_sample_pose = Pose2D(
                    self.pose.x_m,
                    self.pose.y_m,
                    self.pose.heading_rad,
                )

        if 0 < sensors.distance_cm <= self.config.obstacle.trigger_distance_cm * 2.0:
            distance_m = sensors.distance_cm / 100.0
            self.obstacle_map.append(
                MapPoint(
                    x_m=self.pose.x_m + distance_m * math.cos(self.pose.heading_rad),
                    y_m=self.pose.y_m + distance_m * math.sin(self.pose.heading_rad),
                    timestamp=now,
                    kind="obstacle",
                )
            )

        if len(self.line_map) > geo.max_map_points:
            self.line_map = self.line_map[-geo.max_map_points :]
        if len(self.obstacle_map) > geo.max_map_points:
            self.obstacle_map = self.obstacle_map[-geo.max_map_points :]

    def _line_detected(self, infrared_code: int) -> bool:
        if infrared_code == 7:
            return False
        if infrared_code == 0 and not self.config.line.code_zero_is_center:
            return False
        return infrared_code in (0, 1, 2, 3, 4, 5, 6)

    def _path_to_home_blocked(self) -> bool:
        if self._distance_to_home() < 0.4:
            return False

        for point in self.obstacle_map:
            distance_to_segment = self._distance_point_to_segment(
                px=point.x_m,
                py=point.y_m,
                ax=self.pose.x_m,
                ay=self.pose.y_m,
                bx=self.home_pose.x_m,
                by=self.home_pose.y_m,
            )
            distance_from_robot = self._distance_between(
                self.pose.x_m,
                self.pose.y_m,
                point.x_m,
                point.y_m,
            )
            if (
                distance_to_segment <= self.config.geo.obstacle_block_radius_m
                and distance_from_robot <= 1.2
            ):
                return True
        return False

    def _integrate_pose(self) -> None:
        now = time.monotonic()
        dt = now - self._last_motion_ts
        self._last_motion_ts = now

        if dt <= 0.0:
            return

        dt = min(dt, 0.2)
        geo = self.config.geo
        left_mps = self._cmd_left * geo.duty_to_mps
        right_mps = self._cmd_right * geo.duty_to_mps

        v = 0.5 * (left_mps + right_mps)
        omega = (right_mps - left_mps) / max(geo.wheel_base_m, 0.001)

        self.pose.heading_rad = self._normalize_angle(
            self.pose.heading_rad + omega * dt
        )
        self.pose.x_m += v * math.cos(self.pose.heading_rad) * dt
        self.pose.y_m += v * math.sin(self.pose.heading_rad) * dt

    def _drive(self, left: int, right: int) -> None:
        self.adapter.set_wheels(int(left), int(right))
        self._cmd_left = int(left)
        self._cmd_right = int(right)

    def _stop_drive(self) -> None:
        self._drive(0, 0)

    def _distance_to_home(self) -> float:
        return self._distance_between(
            self.pose.x_m,
            self.pose.y_m,
            self.home_pose.x_m,
            self.home_pose.y_m,
        )

    @staticmethod
    def _distance_between(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _distance_point_to_segment(
        px: float,
        py: float,
        ax: float,
        ay: float,
        bx: float,
        by: float,
    ) -> float:
        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay

        ab_len_sq = abx * abx + aby * aby
        if ab_len_sq <= 1e-9:
            return math.hypot(px - ax, py - ay)

        t = (apx * abx + apy * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))

        cx = ax + t * abx
        cy = ay + t * aby
        return math.hypot(px - cx, py - cy)
