from pathlib import Path
import sys
import time

repo_root = Path(__file__).resolve().parents[1]
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from challenge.config import MissionConfig
from challenge.interfaces import CarAdapter
from challenge.mission import AutonomousMission


def main() -> None:
    cfg = MissionConfig()
    adapter = CarAdapter()
    mission = AutonomousMission(adapter=adapter, config=cfg)
    status_interval_s = 1.0
    last_status_ts = 0.0

    print("[challenge] Starting autonomous setup loop...")
    print("[challenge] This is a setup scaffold, not final competition logic.")
    print("[challenge] status updates enabled (1.0s)")

    adapter.start()
    try:
        while True:
            mission.run_once()

            now = time.monotonic()
            if now - last_status_ts >= status_interval_s:
                sensors = adapter.read_sensors()
                print(
                    "[challenge][status] state=%s ir=%s distance_cm=%.1f ball=%s"
                    % (
                        mission.state.value,
                        sensors.infrared_code,
                        sensors.distance_cm,
                        int(sensors.ball_visible),
                    )
                )
                last_status_ts = now

            time.sleep(cfg.cycle_sleep_s)
    except KeyboardInterrupt:
        print("[challenge] Stopping...")
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()
