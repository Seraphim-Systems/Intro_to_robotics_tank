"""Grand Factory Challenge autonomous mission.

Single-process, single-loop FSM controller that reuses existing Freenove
Code/Server modules for motors, servos, IR, ultrasonic, and camera.
"""

from __future__ import annotations

import enum
import importlib
import importlib.util
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


def _pip_install(package: str) -> bool:
    commands = [
        [sys.executable, "-m", "pip", "install", "--user", package],
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--break-system-packages",
            package,
        ],
    ]
    for command in commands:
        try:
            subprocess.run(command, check=True)
            return True
        except subprocess.CalledProcessError:
            continue
    return False


def _import_or_install(module_name: str, pip_package: str):
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name != module_name:
            raise

        print(
            f"[challenge] Missing module '{module_name}', attempting auto-install ({pip_package})..."
        )
        if not _pip_install(pip_package):
            raise RuntimeError(
                f"Unable to auto-install required dependency '{module_name}'. "
                "Run 'python3 Code/setup.py' and retry."
            ) from exc
        return importlib.import_module(module_name)


cv2 = _import_or_install("cv2", "opencv-python")
np = _import_or_install("numpy", "numpy")


REPO_ROOT = Path(__file__).resolve().parents[1]
SERVER_DIR = REPO_ROOT / "Code" / "Server"
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))


def _load_module(file_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module from {file_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_car_module = _load_module(SERVER_DIR / "car.py", "grand_factory_car_module")
_camera_module = _load_module(SERVER_DIR / "camera.py", "grand_factory_camera_module")
Car = _car_module.Car
Camera = _camera_module.Camera


class MissionState(enum.Enum):
    FOLLOW_LINE = "FOLLOW_LINE"
    EVADE_OBSTACLE = "EVADE_OBSTACLE"
    SEARCH_BALL = "SEARCH_BALL"
    APPROACH_BALL = "APPROACH_BALL"
    GRAB_BALL = "GRAB_BALL"
    NAVIGATE_TO_CENTER = "NAVIGATE_TO_CENTER"
    DROP_BALL = "DROP_BALL"


@dataclass
class ControlConfig:
    loop_dt_s: float = 0.04
    vision_period_s: float = 0.12
    obstacle_cm: float = 18.0
    obstacle_valid_min_cm: float = 2.0
    obstacle_valid_max_cm: float = 120.0

    line_base_speed: int = 1250
    line_turn_delta: int = 950
    line_search_spin_speed: int = 850
    line_lost_timeout_s: float = 0.8

    ball_center_tolerance_px: int = 35
    approach_base_speed: int = 850
    approach_turn_speed: int = 650
    ball_radius_grab_px: float = 34.0
    grab_ultrasonic_cm: float = 9.0
    search_ball_timeout_s: float = 6.0

    center_cross_confirm_cycles: int = 4
    navigate_timeout_s: float = 14.0


@dataclass
class BallObservation:
    visible: bool
    center_x: float = 0.0
    center_y: float = 0.0
    radius_px: float = 0.0
    approx_distance_cm: float = -1.0


class GrandFactoryAutonomy:
    def __init__(self, cfg: ControlConfig):
        self.cfg = cfg
        self.car = Car()
        self.camera = Camera(stream_size=(400, 300))

        self.state = MissionState.FOLLOW_LINE
        self.state_started = time.monotonic()
        self.last_line_seen = time.monotonic()
        self.last_vision_ts = 0.0
        self.ball = BallObservation(False)
        self.ball_carrying = False
        self.center_cross_hits = 0

        self.evade_phase = 0
        self.evade_phase_started = 0.0

        self.grab_started = False
        self.drop_started = False

    def start(self) -> None:
        self.camera.start_stream()
        self._safe_pose_arm()
        self._transition(MissionState.FOLLOW_LINE)

    def stop(self) -> None:
        try:
            self.car.motor.setMotorModel(0, 0)
        finally:
            self.camera.stop_stream()
            self.camera.close()
            self.car.close()

    def run_once(self) -> None:
        now = time.monotonic()
        ir_code = self.car.infrared.read_all_infrared()
        sonic_cm = self.car.sonic.get_distance()

        if now - self.last_vision_ts >= self.cfg.vision_period_s:
            self.ball = self._detect_red_ball()
            self.last_vision_ts = now

        obstacle_detected = self._is_obstacle(sonic_cm)
        if obstacle_detected and self.state not in (
            MissionState.GRAB_BALL,
            MissionState.DROP_BALL,
        ):
            self._transition(MissionState.EVADE_OBSTACLE)

        if self.state == MissionState.FOLLOW_LINE:
            self._state_follow_line(ir_code)
        elif self.state == MissionState.EVADE_OBSTACLE:
            self._state_evade_obstacle()
        elif self.state == MissionState.SEARCH_BALL:
            self._state_search_ball()
        elif self.state == MissionState.APPROACH_BALL:
            self._state_approach_ball(sonic_cm)
        elif self.state == MissionState.GRAB_BALL:
            self._state_grab_ball()
        elif self.state == MissionState.NAVIGATE_TO_CENTER:
            self._state_navigate_to_center(ir_code)
        elif self.state == MissionState.DROP_BALL:
            self._state_drop_ball()

    def _state_follow_line(self, ir_code: int) -> None:
        if self.ball.visible and not self.ball_carrying:
            self._transition(MissionState.SEARCH_BALL)
            return

        if ir_code in (2, 5):
            self.last_line_seen = time.monotonic()
            self.car.motor.setMotorModel(
                self.cfg.line_base_speed, self.cfg.line_base_speed
            )
        elif ir_code in (4, 6):
            self.last_line_seen = time.monotonic()
            self.car.motor.setMotorModel(
                self.cfg.line_base_speed - self.cfg.line_turn_delta,
                self.cfg.line_base_speed + self.cfg.line_turn_delta,
            )
        elif ir_code in (1, 3):
            self.last_line_seen = time.monotonic()
            self.car.motor.setMotorModel(
                self.cfg.line_base_speed + self.cfg.line_turn_delta,
                self.cfg.line_base_speed - self.cfg.line_turn_delta,
            )
        elif ir_code == 7:
            self.last_line_seen = time.monotonic()
            self.car.motor.setMotorModel(
                self.cfg.line_base_speed, self.cfg.line_base_speed
            )
        else:
            if time.monotonic() - self.last_line_seen > self.cfg.line_lost_timeout_s:
                self.car.motor.setMotorModel(-700, 700)
            else:
                self.car.motor.setMotorModel(
                    self.cfg.line_base_speed, self.cfg.line_base_speed
                )

    def _state_evade_obstacle(self) -> None:
        now = time.monotonic()
        if self.evade_phase == 0:
            self.evade_phase = 1
            self.evade_phase_started = now
            self.car.motor.setMotorModel(-1200, -1200)
            return

        phase_dt = now - self.evade_phase_started
        if self.evade_phase == 1 and phase_dt >= 0.35:
            self.evade_phase = 2
            self.evade_phase_started = now
            self.car.motor.setMotorModel(-1300, 1300)
        elif self.evade_phase == 2 and phase_dt >= 0.50:
            self.evade_phase = 3
            self.evade_phase_started = now
            self.car.motor.setMotorModel(1200, 1200)
        elif self.evade_phase == 3 and phase_dt >= 0.35:
            self.evade_phase = 4
            self.evade_phase_started = now
            self.car.motor.setMotorModel(1300, -1300)
        elif self.evade_phase == 4 and phase_dt >= 0.50:
            self.evade_phase = 0
            self.car.motor.setMotorModel(0, 0)
            if self.ball.visible and not self.ball_carrying:
                self._transition(MissionState.SEARCH_BALL)
            else:
                self._transition(MissionState.FOLLOW_LINE)

    def _state_search_ball(self) -> None:
        if self.ball.visible:
            self._transition(MissionState.APPROACH_BALL)
            return

        if time.monotonic() - self.state_started > self.cfg.search_ball_timeout_s:
            self._transition(MissionState.FOLLOW_LINE)
            return

        self.car.motor.setMotorModel(
            -self.cfg.line_search_spin_speed, self.cfg.line_search_spin_speed
        )

    def _state_approach_ball(self, sonic_cm: float) -> None:
        if not self.ball.visible:
            self._transition(MissionState.SEARCH_BALL)
            return

        frame_center_x = 200.0
        x_error = self.ball.center_x - frame_center_x

        if abs(x_error) > self.cfg.ball_center_tolerance_px:
            if x_error > 0:
                self.car.motor.setMotorModel(
                    self.cfg.approach_turn_speed, -self.cfg.approach_turn_speed
                )
            else:
                self.car.motor.setMotorModel(
                    -self.cfg.approach_turn_speed, self.cfg.approach_turn_speed
                )
            return

        close_by_ultrasonic = (
            self._is_valid_distance(sonic_cm)
            and sonic_cm <= self.cfg.grab_ultrasonic_cm
        )
        close_by_vision = self.ball.radius_px >= self.cfg.ball_radius_grab_px
        if close_by_ultrasonic or close_by_vision:
            self.car.motor.setMotorModel(0, 0)
            self._transition(MissionState.GRAB_BALL)
            return

        self.car.motor.setMotorModel(
            self.cfg.approach_base_speed, self.cfg.approach_base_speed
        )

    def _state_grab_ball(self) -> None:
        if not self.grab_started:
            self.car.motor.setMotorModel(0, 0)
            self.car.set_mode_clamp(1)
            self.grab_started = True

        self.car.mode_clamp()
        if self.car.get_mode_clamp() == 0:
            self.ball_carrying = True
            self.grab_started = False
            self._transition(MissionState.NAVIGATE_TO_CENTER)

    def _state_navigate_to_center(self, ir_code: int) -> None:
        if ir_code == 7:
            self.center_cross_hits += 1
        else:
            self.center_cross_hits = max(0, self.center_cross_hits - 1)

        if self.center_cross_hits >= self.cfg.center_cross_confirm_cycles:
            self.car.motor.setMotorModel(0, 0)
            self._transition(MissionState.DROP_BALL)
            return

        if time.monotonic() - self.state_started > self.cfg.navigate_timeout_s:
            self.car.motor.setMotorModel(0, 0)
            self._transition(MissionState.DROP_BALL)
            return

        self._state_follow_line(ir_code)

    def _state_drop_ball(self) -> None:
        if not self.drop_started:
            self.car.motor.setMotorModel(0, 0)
            self.car.set_mode_clamp(2)
            self.drop_started = True

        self.car.mode_clamp()
        if self.car.get_mode_clamp() == 0:
            self.drop_started = False
            self.ball_carrying = False
            self.center_cross_hits = 0
            self.car.motor.setMotorModel(-1000, -1000)
            time.sleep(0.25)
            self.car.motor.setMotorModel(0, 0)
            self._transition(MissionState.FOLLOW_LINE)

    def _transition(self, new_state: MissionState) -> None:
        if new_state != self.state:
            print(f"[autonomy] {self.state.value} -> {new_state.value}")
            self.state = new_state
            self.state_started = time.monotonic()
            if new_state != MissionState.EVADE_OBSTACLE:
                self.evade_phase = 0

    def _safe_pose_arm(self) -> None:
        self.car.servo.setServoAngle("0", 90)
        self.car.servo.setServoAngle("1", 140)

    def _is_valid_distance(self, distance_cm: float) -> bool:
        return (
            self.cfg.obstacle_valid_min_cm
            <= distance_cm
            <= self.cfg.obstacle_valid_max_cm
        )

    def _is_obstacle(self, distance_cm: float) -> bool:
        return (
            self._is_valid_distance(distance_cm) and distance_cm <= self.cfg.obstacle_cm
        )

    def _detect_red_ball(self) -> BallObservation:
        try:
            frame_bytes = self.camera.get_frame()
            if not frame_bytes:
                return BallObservation(False)

            frame = cv2.imdecode(
                np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                return BallObservation(False)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 110, 70], dtype=np.uint8)
            upper_red1 = np.array([12, 255, 255], dtype=np.uint8)
            lower_red2 = np.array([168, 110, 70], dtype=np.uint8)
            upper_red2 = np.array([179, 255, 255], dtype=np.uint8)

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            kernel = np.ones((3, 3), dtype=np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                return BallObservation(False)

            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < 220:
                return BallObservation(False)

            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius < 6:
                return BallObservation(False)

            approx_distance_cm = 1660.0 / max(2.0 * radius, 1.0)
            return BallObservation(
                visible=True,
                center_x=float(x),
                center_y=float(y),
                radius_px=float(radius),
                approx_distance_cm=float(approx_distance_cm),
            )
        except (cv2.error, TypeError, ValueError):
            return BallObservation(False)
