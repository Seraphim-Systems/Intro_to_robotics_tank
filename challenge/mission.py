import enum
import time

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
        self.adapter.set_wheels(-700, 700)
        time.sleep(0.2)
        self.state = MissionState.FOLLOW_LINE
