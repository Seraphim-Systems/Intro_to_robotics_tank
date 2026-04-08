from collections.abc import Callable

from challenge.program_base import BaseProgram
from challenge.programs_impl import LineBallSetupProgram


ProgramFactory = Callable[[], BaseProgram]


def available_programs() -> dict[str, ProgramFactory]:
    """Program catalog that can later be exposed through an API/web layer."""

    return {
        "line_ball_setup": LineBallSetupProgram,
    }


def create_program(program_name: str) -> BaseProgram:
    catalog = available_programs()
    if program_name not in catalog:
        valid = ", ".join(sorted(catalog.keys()))
        raise ValueError(f"Unknown program '{program_name}'. Available: {valid}")
    return catalog[program_name]()
