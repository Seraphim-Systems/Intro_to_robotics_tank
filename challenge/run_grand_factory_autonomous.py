from pathlib import Path
import sys
import time

repo_root = Path(__file__).resolve().parents[1]
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from challenge.grand_factory_autonomous import ControlConfig, GrandFactoryAutonomy


def main() -> None:
    cfg = ControlConfig()
    mission = GrandFactoryAutonomy(cfg)

    print("[challenge] Starting grand factory autonomous loop...")
    mission.start()
    try:
        while True:
            mission.run_once()
            time.sleep(cfg.loop_dt_s)
    except KeyboardInterrupt:
        print("[challenge] Stopping...")
    finally:
        mission.stop()


if __name__ == "__main__":
    main()
