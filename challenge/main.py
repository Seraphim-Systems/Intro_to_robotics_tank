import argparse
import importlib.util
from pathlib import Path
import select
import sys
import time
from typing import Optional


repo_root = Path(__file__).resolve().parents[1]
server_dir = repo_root / "Code" / "Server"
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))
if str(server_dir) not in sys.path:
    sys.path.insert(0, str(server_dir))

from challenge.mission import ChallengeMission, MissionConfig  # noqa: E402


def load_car_class():
    car_path = server_dir / "car.py"
    spec = importlib.util.spec_from_file_location("challenge_server_car", car_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load car module from {car_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "Car"):
        raise RuntimeError(f"Car class not found in {car_path}")
    return module.Car


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run challenge mission (car.py-based) with optional manual WASD controls"
    )
    parser.add_argument("--obstacle-cm", type=float, default=18.0)
    parser.add_argument("--pickup-cm", type=float, default=8.0)
    parser.add_argument("--home-radius-m", type=float, default=0.22)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--loop-sleep", type=float, default=0.05)
    parser.add_argument("--line-crawl-speed", type=int, default=260)
    parser.add_argument("--ir-zero-lost", action="store_true")
    return parser.parse_args()


def apply_args(cfg: MissionConfig, args: argparse.Namespace) -> None:
    cfg.obstacle_distance_cm = args.obstacle_cm
    cfg.pickup_distance_cm = args.pickup_cm
    cfg.home_radius_m = max(0.05, args.home_radius_m)
    cfg.loop_sleep_s = max(0.01, args.loop_sleep)
    cfg.line_crawl_speed = max(120, args.line_crawl_speed)
    if args.ir_zero_lost:
        cfg.line_code_zero_is_center = False


def read_command() -> Optional[str]:
    if not sys.stdin or sys.stdin.closed or not sys.stdin.isatty():
        return None

    try:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    except (OSError, ValueError):
        return None

    if not readable:
        return None

    line = sys.stdin.readline()
    if not line:
        return None
    return line.rstrip("\r\n").lower()


def handle_command(command: str, mission: ChallengeMission, cfg: MissionConfig) -> bool:
    step_s = max(0.10, min(0.28, cfg.loop_sleep_s * 4.0))

    if mission.manual_drive_pulse(command, step_s):
        return True

    if command in (" ", "space"):
        mission.manual_pickup_toggle()
        return True

    if command == "home":
        mission.reset_home_anchor()
        print("[challenge] home anchor reset")
        return True

    if command == "status":
        status = mission.get_status()
        print(
            "[challenge] state=%s ir=%s distance_cm=%.1f carrying=%s home_m=%.2f balls=%s obstacles=%s"
            % (
                status["state"],
                status["ir"],
                status["distance_cm"],
                status["carrying"],
                status["home_m"],
                status["balls"],
                status["obstacles"],
            )
        )
        return True

    if command in ("help", "?"):
        print("[challenge] commands: w a s d space home status help")
        return True

    return False


def main() -> None:
    args = parse_args()
    cfg = MissionConfig()
    apply_args(cfg, args)

    Car = load_car_class()
    car = Car()
    mission = ChallengeMission(car=car, config=cfg)
    mission.reset_home_anchor()

    status_interval = max(0.0, args.status_interval)
    last_status = 0.0

    print("[challenge] main started (car.py runtime)")
    print(
        "[challenge] obstacle_cm=%.1f pickup_cm=%.1f home_radius_m=%.2f"
        % (cfg.obstacle_distance_cm, cfg.pickup_distance_cm, cfg.home_radius_m)
    )
    print("[challenge] commands: w a s d space home status help")

    try:
        while True:
            command = read_command()
            manual_handled = False
            if command:
                manual_handled = handle_command(command, mission, cfg)

            if not manual_handled:
                mission.step()

            now = time.monotonic()
            if status_interval > 0 and now - last_status >= status_interval:
                status = mission.get_status()
                print(
                    "[challenge][status] state=%s ir=%s distance_cm=%.1f carrying=%s home_m=%.2f balls=%s obstacles=%s"
                    % (
                        status["state"],
                        status["ir"],
                        status["distance_cm"],
                        status["carrying"],
                        status["home_m"],
                        status["balls"],
                        status["obstacles"],
                    )
                )
                last_status = now

            time.sleep(cfg.loop_sleep_s)
    except KeyboardInterrupt:
        print("[challenge] stopping")
    finally:
        car.close()


if __name__ == "__main__":
    main()
