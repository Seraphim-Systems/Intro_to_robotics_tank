import math
import time
from dataclasses import dataclass
from enum import Enum


@dataclass
class MissionConfig:
    """Single mission configuration object for challenge runtime."""

    loop_sleep_s: float = 0.05
    obstacle_distance_cm: float = 18.0
    pickup_distance_cm: float = 15.0
    home_radius_m: float = 0.22

    # Line-follow and fallback behavior.
    line_code_zero_is_center: bool = True
    line_crawl_speed: int = 260

    # Avoidance motion profile.
    avoid_backup_speed: int = -1200
    avoid_backup_s: float = 0.30
    avoid_turn_left_speed: int = -1000
    avoid_turn_right_speed: int = 1000
    avoid_turn_s: float = 0.34
    avoid_bypass_speed: int = 900
    avoid_bypass_s: float = 0.42
    avoid_return_turn_s: float = 0.28
    avoid_settle_s: float = 0.12

    # Clamp timing.
    pre_open_before_pick: bool = True
    pick_timeout_s: float = 6.0
    drop_timeout_s: float = 4.0

    # Dead-reckoning constants for return-to-start.
    duty_to_mps: float = 0.00022
    wheel_base_m: float = 0.16
    heading_tolerance_rad: float = 0.28


class MissionState(Enum):
    FOLLOW_LINE = "follow_line"
    AVOID_OBSTACLE = "avoid_obstacle"
    PICK_BALL = "pick_ball"
    RETURN_HOME = "return_home"
    DROP_BALL = "drop_ball"


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


@dataclass
class MapPoint:
    x_m: float
    y_m: float
    kind: str


class ChallengeMission:
    """Simple mission that uses only car.py capabilities."""

    def __init__(self, car, config: MissionConfig):
        self.car = car
        self.config = config
        self.state = MissionState.FOLLOW_LINE
        self._carrying_ball = False

        self.pose = Pose2D()
        self.home_pose = Pose2D()

        self._cmd_left = 0
        self._cmd_right = 0
        self._last_motion_ts = time.monotonic()

        self._avoid_phase = ""
        self._avoid_phase_end_ts = 0.0
        self._avoid_resume_state = MissionState.FOLLOW_LINE

        # Permanent memory for observed entities.
        self.obstacle_memory: list[MapPoint] = []
        self.ball_memory: list[MapPoint] = []

    def step(self) -> None:
        self._integrate_pose()
        distance = self._distance_cm()

        if self.state == MissionState.AVOID_OBSTACLE:
            self._step_avoidance()
            return
        if self.state == MissionState.PICK_BALL:
            self._pick_ball()
            return
        if self.state == MissionState.DROP_BALL:
            self._drop_ball()
            return

        if self._is_obstacle(distance):
            self._remember_obstacle(distance)
            self._start_avoidance(resume_state=self.state)
            self._step_avoidance()
            return

        if self.state == MissionState.RETURN_HOME:
            self._return_home_step()
            return

        # FOLLOW_LINE
        if self._is_pickup_distance(distance):
            self._remember_ball_here()
            self.state = MissionState.PICK_BALL
            return

        self._follow_line_continuous()

    def reset_home_anchor(self) -> None:
        self.home_pose = Pose2D(self.pose.x_m, self.pose.y_m, self.pose.heading_rad)

    def is_carrying_ball(self) -> bool:
        return self._carrying_ball

    def set_manual_carrying_state(self, carrying: bool) -> None:
        self._carrying_ball = carrying
        self.state = MissionState.RETURN_HOME if carrying else MissionState.FOLLOW_LINE

    def get_status(self) -> dict[str, float | int | str]:
        return {
            "state": self.state.value,
            "x_m": self.pose.x_m,
            "y_m": self.pose.y_m,
            "heading_deg": math.degrees(self.pose.heading_rad),
            "home_m": self._distance_to_home(),
            "distance_cm": self._distance_cm(),
            "ir": self._read_ir(),
            "carrying": int(self._carrying_ball),
            "balls": len(self.ball_memory),
            "obstacles": len(self.obstacle_memory),
        }

    def manual_drive_pulse(self, key: str, step_s: float) -> bool:
        if key == "w":
            self._drive(900, 900)
        elif key == "s":
            self._drive(-900, -900)
        elif key == "a":
            self._drive(-850, 850)
        elif key == "d":
            self._drive(850, -850)
        else:
            return False

        time.sleep(step_s)
        self._stop_drive()
        return True

    def manual_pickup_toggle(self) -> None:
        if self._carrying_ball:
            self._drop_ball()
            return
        self._pick_ball()

    def _follow_line_continuous(self) -> None:
        ir = self._read_ir()
        if self._is_line_lost(ir):
            self._drive(self.config.line_crawl_speed, self.config.line_crawl_speed)
            return

        previous_flag = getattr(self.car, "infrared_run_stop", False)
        try:
            # Keep mode_infrared for line-follow only; challenge mission handles pickup.
            self.car.infrared_run_stop = True
            self.car.mode_infrared()
        finally:
            self.car.infrared_run_stop = previous_flag

        # Mirror legacy command for dead-reckoning.
        left, right = self._infer_line_command(ir)
        self._cmd_left = left
        self._cmd_right = right

    def _return_home_step(self) -> None:
        if self._distance_to_home() <= self.config.home_radius_m:
            self.state = MissionState.DROP_BALL
            return

        target_heading = math.atan2(
            self.home_pose.y_m - self.pose.y_m,
            self.home_pose.x_m - self.pose.x_m,
        )
        heading_error = self._normalize_angle(target_heading - self.pose.heading_rad)

        if abs(heading_error) > self.config.heading_tolerance_rad:
            if heading_error > 0:
                self._drive(-700, 700)
            else:
                self._drive(700, -700)
            return

        self._drive(850, 850)

    def _pick_ball(self) -> None:
        self._stop_drive()
        if self.config.pre_open_before_pick:
            self._run_clamp(mode=2, timeout_s=self.config.drop_timeout_s)
            time.sleep(0.10)
        self._run_clamp(mode=1, timeout_s=self.config.pick_timeout_s)
        self._carrying_ball = True
        self.state = MissionState.RETURN_HOME

    def _drop_ball(self) -> None:
        self._stop_drive()
        self._run_clamp(mode=2, timeout_s=self.config.drop_timeout_s)
        self._carrying_ball = False
        self.state = MissionState.FOLLOW_LINE

    def _start_avoidance(self, resume_state: MissionState) -> None:
        self.state = MissionState.AVOID_OBSTACLE
        self._avoid_resume_state = resume_state
        self._avoid_phase = "backup"
        self._avoid_phase_end_ts = time.time() + self.config.avoid_backup_s

    def _step_avoidance(self) -> None:
        now = time.time()
        if self._avoid_phase == "backup":
            self._drive(self.config.avoid_backup_speed, self.config.avoid_backup_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "turn"
                self._avoid_phase_end_ts = now + self.config.avoid_turn_s
            return

        if self._avoid_phase == "turn":
            self._drive(
                self.config.avoid_turn_left_speed, self.config.avoid_turn_right_speed
            )
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "bypass"
                self._avoid_phase_end_ts = now + self.config.avoid_bypass_s
            return

        if self._avoid_phase == "bypass":
            self._drive(self.config.avoid_bypass_speed, self.config.avoid_bypass_speed)
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "return"
                self._avoid_phase_end_ts = now + self.config.avoid_return_turn_s
            return

        if self._avoid_phase == "return":
            self._drive(
                self.config.avoid_turn_right_speed, self.config.avoid_turn_left_speed
            )
            if now >= self._avoid_phase_end_ts:
                self._avoid_phase = "settle"
                self._avoid_phase_end_ts = now + self.config.avoid_settle_s
            return

        self._stop_drive()
        if now >= self._avoid_phase_end_ts:
            self._avoid_phase = ""
            self.state = self._avoid_resume_state

    def _run_clamp(self, mode: int, timeout_s: float) -> None:
        self.car.set_mode_clamp(mode)
        deadline = time.monotonic() + timeout_s
        while self.car.get_mode_clamp() == mode:
            self.car.mode_clamp()
            if time.monotonic() >= deadline:
                self.car.set_mode_clamp(0)
                break

    def _remember_obstacle(self, distance_cm: float) -> None:
        distance_m = distance_cm / 100.0
        self.obstacle_memory.append(
            MapPoint(
                x_m=self.pose.x_m + distance_m * math.cos(self.pose.heading_rad),
                y_m=self.pose.y_m + distance_m * math.sin(self.pose.heading_rad),
                kind="obstacle",
            )
        )

    def _remember_ball_here(self) -> None:
        self.ball_memory.append(MapPoint(self.pose.x_m, self.pose.y_m, "ball"))

    def _is_obstacle(self, distance_cm: float) -> bool:
        if self._carrying_ball:
            return 0 < distance_cm <= self.config.obstacle_distance_cm
        # Prefer pickup first when not carrying and close enough.
        if self._is_pickup_distance(distance_cm):
            return False
        return 0 < distance_cm <= self.config.obstacle_distance_cm

    def _is_pickup_distance(self, distance_cm: float) -> bool:
        return 0 < distance_cm <= self.config.pickup_distance_cm

    def _is_line_lost(self, infrared_code: int) -> bool:
        if infrared_code == 7:
            return True
        if infrared_code == 0 and not self.config.line_code_zero_is_center:
            return True
        return False

    def _distance_cm(self) -> float:
        try:
            return float(self.car.sonic.get_distance())
        except Exception:
            return -1.0

    def _read_ir(self) -> int:
        try:
            return int(self.car.infrared.read_all_infrared())
        except Exception:
            return 7

    def _infer_line_command(self, infrared_code: int) -> tuple[int, int]:
        if infrared_code == 2:
            return 1200, 1200
        if infrared_code == 4:
            return -1500, 2500
        if infrared_code == 6:
            return -2000, 4000
        if infrared_code == 1:
            return 2500, -1500
        if infrared_code == 3:
            return 4000, -2000
        if infrared_code == 0 and self.config.line_code_zero_is_center:
            return 1200, 1200
        return self.config.line_crawl_speed, self.config.line_crawl_speed

    def _integrate_pose(self) -> None:
        now = time.monotonic()
        dt = now - self._last_motion_ts
        self._last_motion_ts = now
        if dt <= 0.0:
            return

        dt = min(dt, 0.2)
        left_mps = self._cmd_left * self.config.duty_to_mps
        right_mps = self._cmd_right * self.config.duty_to_mps

        v = 0.5 * (left_mps + right_mps)
        omega = (right_mps - left_mps) / max(self.config.wheel_base_m, 0.001)
        self.pose.heading_rad = self._normalize_angle(
            self.pose.heading_rad + omega * dt
        )
        self.pose.x_m += v * math.cos(self.pose.heading_rad) * dt
        self.pose.y_m += v * math.sin(self.pose.heading_rad) * dt

    def _drive(self, left: int, right: int) -> None:
        self.car.motor.setMotorModel(int(left), int(right))
        self._cmd_left = int(left)
        self._cmd_right = int(right)

    def _stop_drive(self) -> None:
        self._drive(0, 0)

    def _distance_to_home(self) -> float:
        return math.hypot(
            self.home_pose.x_m - self.pose.x_m, self.home_pose.y_m - self.pose.y_m
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
