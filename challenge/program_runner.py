import time

from challenge.program_registry import create_program


def run_program_forever(program_name: str, cycle_sleep_s: float = 0.05) -> None:
    """Run a named program until interrupted."""

    program = create_program(program_name)
    print(f"[challenge] starting program={program_name}")
    program.start()
    try:
        while True:
            program.step()
            time.sleep(cycle_sleep_s)
    except KeyboardInterrupt:
        print("[challenge] keyboard interrupt received")
    finally:
        program.stop()
        print(f"[challenge] stopped program={program_name}")
