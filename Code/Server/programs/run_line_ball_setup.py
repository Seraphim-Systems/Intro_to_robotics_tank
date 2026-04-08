"""Preliminary executable script for the autonomous line+ball setup program.

This script is designed to be callable from future web-server orchestration,
but has no web dependency itself.
"""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from challenge.program_runner import run_program_forever


def main() -> None:
    run_program_forever("line_ball_setup", cycle_sleep_s=0.05)


if __name__ == "__main__":
    main()
