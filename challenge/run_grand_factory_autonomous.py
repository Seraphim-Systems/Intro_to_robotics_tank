import time

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
