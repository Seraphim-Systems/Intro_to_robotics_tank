from abc import ABC, abstractmethod


class BaseProgram(ABC):
    """Base contract for runnable robot programs."""

    name: str = "base"

    @abstractmethod
    def start(self) -> None:
        """Allocate resources and initialize the program."""

    @abstractmethod
    def step(self) -> None:
        """Run one control cycle."""

    @abstractmethod
    def stop(self) -> None:
        """Release resources and stop safely."""
