from dataclasses import dataclass
import importlib
import importlib.util
from pathlib import Path
import os
import shutil
import subprocess
import sys
import time
from typing import Optional

from .config import MissionConfig


_APT_MODULE_PACKAGES = {
    "cv2": "python3-opencv",
    "numpy": "python3-numpy",
    "picamera2": "python3-picamera2",
    "libcamera": "python3-libcamera",
    "gpiozero": "python3-gpiozero",
    "pigpio": "python3-pigpio",
    "lgpio": "python3-lgpio",
}

_PIP_MODULE_PACKAGES = {
    "cv2": "opencv-python",
    "numpy": "numpy",
    "rpi_hardware_pwm": "rpi-hardware-pwm",
}


def _is_root() -> bool:
    try:
        return os.geteuid() == 0
    except AttributeError:
        return False


def _apt_install(package: str) -> bool:
    apt_get = shutil.which("apt-get")
    if apt_get is None:
        return False

    command = [apt_get, "install", "-y", package]
    if not _is_root():
        sudo = shutil.which("sudo")
        if sudo is None:
            return False
        command = [sudo] + command

    try:
        subprocess.run(command, check=True)
        return True
    except subprocess.CalledProcessError:
        return False


def _pip_install(package: str) -> bool:
    if _is_root():
        commands = [
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "--break-system-packages",
                package,
            ],
            [sys.executable, "-m", "pip", "install", package],
        ]
    else:
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


def _import_with_optional_auto_install(
    module_name: str,
    pip_package: str,
    apt_package: Optional[str] = None,
):
    try:
        return importlib.import_module(module_name)
    except (ModuleNotFoundError, ImportError) as exc:
        # If import failed for a nested dependency, do not retry pip blindly.
        missing_name = getattr(exc, "name", module_name)
        if missing_name and missing_name != module_name:
            return None

        auto_install = os.environ.get("CHALLENGE_AUTO_INSTALL_DEPS", "1").lower()
        if auto_install in ("0", "false", "no"):
            return None

        if apt_package is not None:
            print(
                f"[challenge] Missing module '{module_name}', attempting apt install ({apt_package})..."
            )
            if _apt_install(apt_package):
                try:
                    return importlib.import_module(module_name)
                except (ModuleNotFoundError, ImportError):
                    pass

        print(
            f"[challenge] Missing module '{module_name}', attempting pip install ({pip_package})..."
        )
        if not _pip_install(pip_package):
            return None

        try:
            return importlib.import_module(module_name)
        except (ModuleNotFoundError, ImportError):
            return None


def _install_dependency_for_module(module_name: str) -> bool:
    auto_install = os.environ.get("CHALLENGE_AUTO_INSTALL_DEPS", "1").lower()
    if auto_install in ("0", "false", "no"):
        return False

    apt_package = _APT_MODULE_PACKAGES.get(module_name)
    if apt_package is not None:
        print(
            f"[challenge] Missing module '{module_name}', attempting apt install ({apt_package})..."
        )
        if _apt_install(apt_package):
            return True

    pip_package = _PIP_MODULE_PACKAGES.get(module_name)
    if pip_package is not None:
        print(
            f"[challenge] Missing module '{module_name}', attempting pip install ({pip_package})..."
        )
        if _pip_install(pip_package):
            return True

    return False


cv2 = _import_with_optional_auto_install(
    "cv2",
    "opencv-python",
    apt_package="python3-opencv",
)
np = _import_with_optional_auto_install(
    "numpy",
    "numpy",
    apt_package="python3-numpy",
)


@dataclass
class SensorSnapshot:
    infrared_code: int
    distance_cm: float
    ball_visible: bool
    ball_center_x: Optional[float] = None
    ball_radius_px: Optional[float] = None
    marker_visible: bool = False
    marker_center_x: Optional[float] = None
    marker_radius_px: Optional[float] = None
    frame_width_px: Optional[int] = None
    frame_height_px: Optional[int] = None


class CarAdapter:
    """Thin adapter over existing Code/Server classes.

    Keeps challenge code decoupled from legacy server threading implementation.
    """

    def __init__(self, config: Optional[MissionConfig] = None):
        # Deferred module loading keeps setup importable on non-RPi systems.
        server_dir = Path(__file__).resolve().parents[1] / "Code" / "Server"
        if str(server_dir) not in sys.path:
            sys.path.insert(0, str(server_dir))
        car_module = self._load_module(server_dir / "car.py", "challenge_car_module")
        camera_module = self._load_module(
            server_dir / "camera.py", "challenge_camera_module"
        )

        self.config = config or MissionConfig()
        self.car = car_module.Car()
        self.camera = camera_module.Camera(stream_size=(400, 300))
        self._stream_started = False
        self._vision_available = cv2 is not None and np is not None
        self._vision_warning_emitted = False
        self._clamp_pick_timeout_s = 6.0
        self._clamp_drop_timeout_s = 4.0

        self._ball_ranges = [
            (self.config.ball.hsv_lower_1, self.config.ball.hsv_upper_1),
            (self.config.ball.hsv_lower_2, self.config.ball.hsv_upper_2),
        ]
        self._home_marker_ranges: list[
            tuple[tuple[int, int, int], tuple[int, int, int]]
        ] = []

        # Keep servo idle between clamp actions to reduce startup buzzing.
        self._release_servo_pwm()

    @staticmethod
    def _load_module(file_path: Path, module_name: str):
        # Retry once after optional dependency auto-install.
        for _ in range(2):
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Cannot load module from {file_path}")

            module = importlib.util.module_from_spec(spec)
            try:
                spec.loader.exec_module(module)
                return module
            except ModuleNotFoundError as exc:
                missing_module = getattr(exc, "name", None)
                if not missing_module or not _install_dependency_for_module(
                    missing_module
                ):
                    raise

        raise ImportError(f"Cannot load module from {file_path}")

    def start(self) -> None:
        if not self._stream_started:
            self.camera.start_stream()
            self._stream_started = True

    def stop(self) -> None:
        self.stop_motors()
        if self._stream_started:
            self.camera.stop_stream()
            self._stream_started = False
        self.camera.close()
        self.car.close()

    def read_sensors(self) -> SensorSnapshot:
        distance_cm = self.car.sonic.get_distance()
        infrared_code = self.car.infrared.read_all_infrared()
        frame = self._decode_frame()
        frame_height = None
        frame_width = None
        if frame is not None:
            frame_height, frame_width = frame.shape[:2]

        ball_visible, center_x, radius_px = self._detect_ball(frame)
        marker_visible, marker_center_x, marker_radius_px = self._detect_home_marker(
            frame
        )

        return SensorSnapshot(
            infrared_code=infrared_code,
            distance_cm=distance_cm,
            ball_visible=ball_visible,
            ball_center_x=center_x,
            ball_radius_px=radius_px,
            marker_visible=marker_visible,
            marker_center_x=marker_center_x,
            marker_radius_px=marker_radius_px,
            frame_width_px=frame_width,
            frame_height_px=frame_height,
        )

    def set_wheels(self, left: int, right: int) -> None:
        self.car.motor.setMotorModel(left, right)

    def stop_motors(self) -> None:
        self.car.motor.setMotorModel(0, 0)

    def move_servo(self, channel: str, angle: int) -> None:
        self.car.servo.setServoAngle(channel, angle)

    def clamp_pick(self) -> None:
        deadline = time.monotonic() + self._clamp_pick_timeout_s
        self.car.set_mode_clamp(1)
        while self.car.get_mode_clamp() == 1:
            self.car.mode_clamp()
            if time.monotonic() >= deadline:
                print("[challenge] clamp_pick timeout; aborting clamp cycle")
                self.car.set_mode_clamp(0)
                self.stop_motors()
                break
        self._release_servo_pwm()

    def clamp_drop(self) -> None:
        deadline = time.monotonic() + self._clamp_drop_timeout_s
        self.car.set_mode_clamp(2)
        while self.car.get_mode_clamp() == 2:
            self.car.mode_clamp()
            if time.monotonic() >= deadline:
                print("[challenge] clamp_drop timeout; aborting clamp cycle")
                self.car.set_mode_clamp(0)
                self.stop_motors()
                break
        self._release_servo_pwm()

    def is_home_marker_ready(self) -> bool:
        return bool(self._home_marker_ranges)

    def calibrate_home_marker(self, sample_frames: Optional[int] = None) -> bool:
        if not self._vision_available:
            self._emit_vision_warning_once()
            return False

        sample_count = sample_frames or self.config.home.calibration_samples
        collected: list[tuple[int, int, int]] = []

        for _ in range(max(1, sample_count)):
            frame = self._decode_frame()
            if frame is None:
                continue

            roi = self._center_roi(frame, self.config.home.calibration_roi_px)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            pixels = hsv.reshape(-1, 3)
            saturated = pixels[pixels[:, 1] > 40]
            if len(saturated) == 0:
                saturated = pixels

            median = np.median(saturated, axis=0).astype(int)
            collected.append((int(median[0]), int(median[1]), int(median[2])))

        if not collected:
            return False

        hsv_median = np.median(np.array(collected), axis=0).astype(int)
        self._home_marker_ranges = self._build_hsv_ranges(
            hue=int(hsv_median[0]),
            sat=int(hsv_median[1]),
            val=int(hsv_median[2]),
            hue_tolerance=self.config.home.hue_tolerance,
            sat_tolerance=self.config.home.sat_tolerance,
            val_tolerance=self.config.home.val_tolerance,
        )
        return True

    def _decode_frame(self):
        if not self._stream_started:
            return None

        if not self._vision_available:
            self._emit_vision_warning_once()
            return None

        try:
            frame_bytes = self.camera.get_frame()
        except Exception:
            return None

        if frame_bytes is None:
            return None

        encoded = np.frombuffer(frame_bytes, dtype=np.uint8)
        if encoded.size == 0:
            return None

        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        return frame

    def _detect_ball(self, frame):
        if frame is None:
            return False, None, None

        return self._detect_color_blob(
            frame=frame,
            ranges=self._ball_ranges,
            min_contour_area=self.config.ball.min_contour_area_px,
            min_radius=self.config.ball.min_radius_px,
        )

    def _detect_home_marker(self, frame):
        if frame is None or not self._home_marker_ranges:
            return False, None, None

        return self._detect_color_blob(
            frame=frame,
            ranges=self._home_marker_ranges,
            min_contour_area=self.config.home.min_contour_area_px,
            min_radius=self.config.home.min_radius_px,
        )

    def _detect_color_blob(
        self, frame, ranges, min_contour_area: int, min_radius: float
    ):
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = None
        for lower, upper in ranges:
            lower_arr = np.array(lower, dtype=np.uint8)
            upper_arr = np.array(upper, dtype=np.uint8)
            range_mask = cv2.inRange(hsv, lower_arr, upper_arr)
            mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        contours_info = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        if not contours:
            return False, None, None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < float(min_contour_area):
            return False, None, None

        (center, radius) = cv2.minEnclosingCircle(contour)
        center_x = float(center[0])
        radius = float(radius)
        if radius < float(min_radius):
            return False, None, None

        return True, center_x, radius

    @staticmethod
    def _center_roi(frame, roi_size_px: int):
        height, width = frame.shape[:2]
        roi = max(10, int(roi_size_px))
        half = roi // 2
        cx = width // 2
        cy = height // 2

        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(width, cx + half)
        y1 = min(height, cy + half)
        return frame[y0:y1, x0:x1]

    @staticmethod
    def _build_hsv_ranges(
        hue: int,
        sat: int,
        val: int,
        hue_tolerance: int,
        sat_tolerance: int,
        val_tolerance: int,
    ) -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
        h_low = hue - hue_tolerance
        h_high = hue + hue_tolerance
        s_low = max(0, sat - sat_tolerance)
        s_high = min(255, sat + sat_tolerance)
        v_low = max(0, val - val_tolerance)
        v_high = min(255, val + val_tolerance)

        if h_low < 0:
            return [
                ((0, s_low, v_low), (h_high, s_high, v_high)),
                ((180 + h_low, s_low, v_low), (179, s_high, v_high)),
            ]
        if h_high > 179:
            return [
                ((0, s_low, v_low), (h_high - 180, s_high, v_high)),
                ((h_low, s_low, v_low), (179, s_high, v_high)),
            ]
        return [((h_low, s_low, v_low), (h_high, s_high, v_high))]

    def _emit_vision_warning_once(self) -> None:
        if self._vision_warning_emitted:
            return
        self._vision_warning_emitted = True
        print(
            "[challenge] cv2/numpy unavailable; vision detection disabled (run: python3 Code/setup.py)"
        )

    def _release_servo_pwm(self) -> None:
        servo = getattr(self.car, "servo", None)
        if servo is None:
            return

        pwm = getattr(servo, "pwm", None)
        if pwm is None:
            return

        # gpiozero backend: detach to stop hold torque at idle.
        for attr in ("servo1", "servo2", "servo3"):
            servo_obj = getattr(pwm, attr, None)
            if servo_obj is not None and hasattr(servo_obj, "detach"):
                try:
                    servo_obj.detach()
                except Exception:
                    pass

        # pigpio backend: set duty cycle to zero on known channels.
        pigpio_handle = getattr(pwm, "PwmServo", None)
        if pigpio_handle is not None:
            for channel_attr in ("channel1", "channel2", "channel3"):
                channel = getattr(pwm, channel_attr, None)
                if channel is None:
                    continue
                try:
                    pigpio_handle.set_PWM_dutycycle(channel, 0)
                except Exception:
                    pass
