from collections.abc import Callable

from challenge.program_base import BaseProgram
from challenge.programs_impl import LineAvoidPickHomeProgram, LineBallSetupProgram


ProgramFactory = Callable[[], BaseProgram]


def available_programs() -> dict[str, ProgramFactory]:
    """Program catalog that can later be exposed through an API/web layer."""

    return {
        "line_ball_setup": LineBallSetupProgram,
        "line_avoid_pick_home": LineAvoidPickHomeProgram,
    }


def create_program(program_name: str) -> BaseProgram:
    catalog = available_programs()
    if program_name not in catalog:
        valid = ", ".join(sorted(catalog.keys()))
        raise ValueError(f"Unknown program '{program_name}'. Available: {valid}")
    return catalog[program_name]()
