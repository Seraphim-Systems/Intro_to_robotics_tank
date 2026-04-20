from challenge.config import MissionConfig
from challenge.grand_factory_autonomous import ControlConfig, GrandFactoryAutonomy
from challenge.interfaces import CarAdapter
from challenge.mission import AutonomousMission
from challenge.program_base import BaseProgram


class LineBallSetupProgram(BaseProgram):
    """Preliminary autonomous program wrapping mission scaffold."""

    name = "line_ball_setup"

    def __init__(self, config: MissionConfig | None = None):
        self.config = config or MissionConfig()
        self.adapter = CarAdapter()
        self.mission = AutonomousMission(adapter=self.adapter, config=self.config)

    def start(self) -> None:
        self.adapter.start()

    def step(self) -> None:
        self.mission.run_once()

    def stop(self) -> None:
        self.adapter.stop()


class GrandFactoryAutonomousProgram(BaseProgram):
    """Full autonomous grand factory mission program."""

    name = "grand_factory_autonomous"

    def __init__(self, config: ControlConfig | None = None):
        self.config = config or ControlConfig()
        self.mission = GrandFactoryAutonomy(self.config)

    def start(self) -> None:
        self.mission.start()

    def step(self) -> None:
        self.mission.run_once()

    def stop(self) -> None:
        self.mission.stop()
