import enum
import time
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
        if ir in (2, 5):
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


class LineAvoidBallHomeMission:
    """Priority-driven autonomous loop for line/avoid/pick/home-drop behavior."""

    def __init__(self, adapter: CarAdapter, config: MissionConfig):
        self.adapter = adapter
        self.config = config
        self.state = HomeCycleState.FOLLOW_LINE

        self._last_line_seen_ts = time.time()
        self._ball_last_seen_ts = 0.0
        self._home_last_seen_ts = 0.0
        self._carrying_ball = False

        self._avoid_phase = ""
        self._avoid_phase_end_ts = 0.0
        self._avoid_resume_state = HomeCycleState.FOLLOW_LINE

    def run_once(self) -> None:
        sensors = self.adapter.read_sensors()

        if self.state == HomeCycleState.AVOID_OBSTACLE:
            self._handle_avoid_obstacle()
            return

        if self._should_start_avoidance(sensors):
            if self.state in (HomeCycleState.FOLLOW_LINE, HomeCycleState.RETURN_HOME):
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

    def _should_start_avoidance(self, sensors: SensorSnapshot) -> bool:
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
            self.adapter.set_wheels(cfg.backup_speed, cfg.backup_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "turn"
                self._avoid_phase_end_ts = now + cfg.turn_duration_s
            return

        if self._avoid_phase == "turn":
            self.adapter.set_wheels(cfg.turn_left_speed, cfg.turn_right_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "settle"
                self._avoid_phase_end_ts = now + cfg.settle_duration_s
            return

        self.adapter.stop_motors()
        if now >= self._avoid_phase_end_ts:
            self._avoid_phase = ""
            self.state = self._avoid_resume_state

    def _handle_follow_line(self, sensors: SensorSnapshot) -> None:
        if sensors.ball_visible:
            self._ball_last_seen_ts = time.time()
            self.adapter.stop_motors()
            self.state = HomeCycleState.APPROACH_BALL
            return

        if self._apply_line_follow_drive(sensors.infrared_code):
            self._last_line_seen_ts = time.time()
            return

        if time.time() - self._last_line_seen_ts > self.config.line.lost_line_timeout_s:
            self.adapter.stop_motors()
            self.state = HomeCycleState.RECOVER

    def _handle_approach_ball(self, sensors: SensorSnapshot) -> None:
        now = time.time()
        ball_cfg = self.config.ball

        if sensors.ball_visible:
            self._ball_last_seen_ts = now
        elif now - self._ball_last_seen_ts > ball_cfg.lost_target_timeout_s:
            self.adapter.stop_motors()
            self.state = HomeCycleState.FOLLOW_LINE
            return

        if (
            ball_cfg.valid_distance_min_cm
            <= sensors.distance_cm
            <= ball_cfg.approach_stop_distance_cm
        ):
            self.adapter.stop_motors()
            self.state = HomeCycleState.PICK_BALL
            return

        if sensors.ball_visible and sensors.ball_center_x is not None:
            self._steer_toward_target(
                center_x=sensors.ball_center_x,
                frame_width=sensors.frame_width_px,
                base_speed=ball_cfg.approach_speed,
                steer_delta=ball_cfg.steer_speed_delta,
                center_tolerance=ball_cfg.center_tolerance_px,
            )
            return

        self.adapter.set_wheels(-ball_cfg.search_spin_speed, ball_cfg.search_spin_speed)

    def _handle_pick_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self.adapter.stop_motors()
        self.adapter.clamp_pick()
        self._carrying_ball = True
        self._home_last_seen_ts = time.time()
        self.state = HomeCycleState.RETURN_HOME

    def _handle_return_home(self, sensors: SensorSnapshot) -> None:
        now = time.time()
        home_cfg = self.config.home

        if sensors.marker_visible:
            self._home_last_seen_ts = now

            if (
                home_cfg.valid_distance_min_cm
                <= sensors.distance_cm
                <= home_cfg.drop_distance_cm
            ):
                self.adapter.stop_motors()
                self.state = HomeCycleState.DROP_BALL
                return

            if sensors.marker_center_x is not None:
                self._steer_toward_target(
                    center_x=sensors.marker_center_x,
                    frame_width=sensors.frame_width_px,
                    base_speed=home_cfg.approach_speed,
                    steer_delta=home_cfg.steer_speed_delta,
                    center_tolerance=home_cfg.center_tolerance_px,
                )
                return

        if now - self._home_last_seen_ts <= home_cfg.lost_target_timeout_s:
            self.adapter.set_wheels(
                -home_cfg.search_spin_speed, home_cfg.search_spin_speed
            )
            return

        if self._apply_line_follow_drive(sensors.infrared_code):
            self._last_line_seen_ts = now
            return

        if now - self._last_line_seen_ts > self.config.line.lost_line_timeout_s:
            self.state = HomeCycleState.RECOVER

    def _handle_drop_ball(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self.adapter.stop_motors()
        self.adapter.clamp_drop()
        self._carrying_ball = False
        self._last_line_seen_ts = time.time()
        self.state = HomeCycleState.FOLLOW_LINE

    def _handle_recover(self, sensors: SensorSnapshot) -> None:
        _ = sensors
        self.adapter.set_wheels(-450, 450)
        time.sleep(0.12)
        self.adapter.stop_motors()
        self._last_line_seen_ts = time.time()
        if self._carrying_ball:
            self.state = HomeCycleState.RETURN_HOME
        else:
            self.state = HomeCycleState.FOLLOW_LINE

    def _apply_line_follow_drive(self, infrared_code: int) -> bool:
        if infrared_code in (2, 5):
            self.adapter.set_wheels(
                self.config.line.base_speed,
                self.config.line.base_speed,
            )
            return True

        if infrared_code in (4, 6):
            self.adapter.set_wheels(
                self.config.line.base_speed - self.config.line.turn_speed_delta,
                self.config.line.base_speed + self.config.line.turn_speed_delta,
            )
            return True

        if infrared_code in (1, 3):
            self.adapter.set_wheels(
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
    ) -> None:
        if frame_width is None or frame_width <= 0:
            self.adapter.set_wheels(base_speed, base_speed)
            return

        mid_x = frame_width / 2.0
        error = center_x - mid_x

        if abs(error) <= center_tolerance:
            self.adapter.set_wheels(base_speed, base_speed)
        elif error < 0:
            self.adapter.set_wheels(base_speed - steer_delta, base_speed + steer_delta)
        else:
            self.adapter.set_wheels(base_speed + steer_delta, base_speed - steer_delta)
