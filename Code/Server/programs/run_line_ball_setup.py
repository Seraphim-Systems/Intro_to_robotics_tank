"""Launcher for simplified challenge runtime."""

import importlib
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def main() -> None:
    run_challenge_main = importlib.import_module("challenge.main").main
    run_challenge_main()


if __name__ == "__main__":
    main()
