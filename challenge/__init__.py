"""Challenge package for autonomous mission setup and program orchestration."""

__all__ = ["available_programs", "create_program"]


def available_programs():
    from challenge.program_registry import available_programs as _available_programs

    return _available_programs()


def create_program(program_name: str):
    from challenge.program_registry import create_program as _create_program

    return _create_program(program_name)
