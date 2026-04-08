from dataclasses import dataclass
from pathlib import Path
import importlib.util
import sys
from typing import Optional


@dataclass
class SensorSnapshot:
    infrared_code: int
    distance_cm: float
    ball_visible: bool
    ball_center_x: Optional[float] = None
    ball_radius_px: Optional[float] = None


class CarAdapter:
    """Thin adapter over existing Code/Server classes.

    Keeps challenge code decoupled from legacy server threading implementation.
    """

    def __init__(self):
        # Deferred module loading keeps setup importable on non-RPi systems.
        server_dir = Path(__file__).resolve().parents[1] / "Code" / "Server"
        if str(server_dir) not in sys.path:
            sys.path.insert(0, str(server_dir))
        car_module = self._load_module(server_dir / "car.py", "challenge_car_module")
        camera_module = self._load_module(
            server_dir / "camera.py", "challenge_camera_module"
        )

        self.car = car_module.Car()
        self.camera = camera_module.Camera(stream_size=(400, 300))
        self._stream_started = False

    @staticmethod
    def _load_module(file_path: Path, module_name: str):
        spec = importlib.util.spec_from_file_location(module_name, file_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load module from {file_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

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
        ball_visible, center_x, radius_px = self._detect_ball_stub()
        return SensorSnapshot(
            infrared_code=infrared_code,
            distance_cm=distance_cm,
            ball_visible=ball_visible,
            ball_center_x=center_x,
            ball_radius_px=radius_px,
        )

    def set_wheels(self, left: int, right: int) -> None:
        self.car.motor.setMotorModel(left, right)

    def stop_motors(self) -> None:
        self.car.motor.setMotorModel(0, 0)

    def move_servo(self, channel: str, angle: int) -> None:
        self.car.servo.setServoAngle(channel, angle)

    def clamp_pick(self) -> None:
        self.car.set_mode_clamp(1)
        while self.car.get_mode_clamp() == 1:
            self.car.mode_clamp()

    def clamp_drop(self) -> None:
        self.car.set_mode_clamp(2)
        while self.car.get_mode_clamp() == 2:
            self.car.mode_clamp()

    def _detect_ball_stub(self):
        """Ball detector placeholder.

        TODO:
        - Pull frame via self.camera.get_frame()
        - Decode JPEG and run segmentation/detection
        - Return (visible, center_x, radius_px)
        """
        return False, None, None
