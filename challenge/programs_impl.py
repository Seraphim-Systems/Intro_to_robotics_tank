from challenge.config import MissionConfig
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
