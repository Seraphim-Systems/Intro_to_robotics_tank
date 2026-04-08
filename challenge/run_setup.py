import time

from challenge.config import MissionConfig
from challenge.interfaces import CarAdapter
from challenge.mission import AutonomousMission


def main() -> None:
    cfg = MissionConfig()
    adapter = CarAdapter()
    mission = AutonomousMission(adapter=adapter, config=cfg)

    print("[challenge] Starting autonomous setup loop...")
    print("[challenge] This is a setup scaffold, not final competition logic.")

    adapter.start()
    try:
        while True:
            mission.run_once()
            time.sleep(cfg.cycle_sleep_s)
    except KeyboardInterrupt:
        print("[challenge] Stopping...")
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()
