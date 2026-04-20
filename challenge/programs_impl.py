from typing import TYPE_CHECKING

from challenge.config import MissionConfig
from challenge.interfaces import CarAdapter
from challenge.mission import AutonomousMission, LineAvoidBallHomeMission
from challenge.program_base import BaseProgram


if TYPE_CHECKING:
    from challenge.grand_factory_autonomous import ControlConfig, GrandFactoryAutonomy


class LineBallSetupProgram(BaseProgram):
    """Preliminary autonomous program wrapping mission scaffold."""

    name = "line_ball_setup"

    def __init__(self, config: MissionConfig | None = None):
        self.config = config or MissionConfig()
        self.adapter = CarAdapter(config=self.config)
        self.mission = AutonomousMission(adapter=self.adapter, config=self.config)

    def start(self) -> None:
        self.adapter.start()

    def step(self) -> None:
        self.mission.run_once()

    def stop(self) -> None:
        self.adapter.stop()


class LineAvoidPickHomeProgram(BaseProgram):
    """Autonomous line/avoid/pick/home-drop cycle program."""

    name = "line_avoid_pick_home"

    def __init__(
        self,
        config: MissionConfig | None = None,
        calibrate_home_on_start: bool = False,
        calibration_samples: int | None = None,
    ):
        self.config = config or MissionConfig()
        self.adapter = CarAdapter(config=self.config)
        self.mission = LineAvoidBallHomeMission(
            adapter=self.adapter, config=self.config
        )
        self.calibrate_home_on_start = calibrate_home_on_start
        self.calibration_samples = calibration_samples

    def start(self) -> None:
        self.adapter.start()
        if self.calibrate_home_on_start:
            ok = self.adapter.calibrate_home_marker(self.calibration_samples)
            if not ok:
                print(
                    "[challenge] home marker calibration failed; marker return disabled"
                )

    def step(self) -> None:
        self.mission.run_once()

    def stop(self) -> None:
        self.adapter.stop()


class GrandFactoryAutonomousProgram(BaseProgram):
    """Full autonomous grand factory mission program."""

    name = "grand_factory_autonomous"

    def __init__(self, config: "ControlConfig | None" = None):
        try:
            from challenge.grand_factory_autonomous import (
                ControlConfig,
                GrandFactoryAutonomy,
            )
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                f"Missing dependency '{exc.name}' for grand_factory_autonomous. "
                "Run 'python3 Code/setup.py' to install dependencies."
            ) from exc

        self.config = config or ControlConfig()
        self.mission = GrandFactoryAutonomy(self.config)

    def start(self) -> None:
        self.mission.start()

    def step(self) -> None:
        self.mission.run_once()

    def stop(self) -> None:
        self.mission.stop()
