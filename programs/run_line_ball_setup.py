"""Preliminary executable script for the autonomous line+ball setup program.

This script is designed to be callable from future web-server orchestration,
but has no web dependency itself.
"""

from challenge.program_runner import run_program_forever


def main() -> None:
    run_program_forever("line_ball_setup", cycle_sleep_s=0.05)


if __name__ == "__main__":
    main()
