import argparse
from pathlib import Path
import select
import sys
import time
from typing import Optional

repo_root = Path(__file__).resolve().parents[1]
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from challenge.config import MissionConfig  # noqa: E402
from challenge.interfaces import CarAdapter  # noqa: E402
from challenge.mission import LineAvoidBallHomeMission  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run autonomous line-follow + avoid + red-ball pickup + home drop loop"
    )
    parser.add_argument(
        "--obstacle-cm",
        type=float,
        default=15.0,
        help="Sonar distance threshold in cm that triggers obstacle avoidance",
    )
    parser.add_argument(
        "--pickup-cm",
        type=float,
        default=8.0,
        help="Sonar stop distance in cm before clamp pickup",
    )
    parser.add_argument(
        "--home-drop-cm",
        type=float,
        default=10.0,
        help="Sonar stop distance in cm before clamp drop at home marker",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=None,
        help="Number of frames sampled during home marker calibration",
    )
    parser.add_argument(
        "--skip-calibration",
        action="store_true",
        help="Skip startup home marker calibration",
    )
    parser.add_argument(
        "--cycle-sleep",
        type=float,
        default=None,
        help="Control-loop sleep in seconds",
    )
    parser.add_argument(
        "--line-base-speed",
        type=int,
        default=None,
        help="Base wheel speed during line-follow",
    )
    parser.add_argument(
        "--line-turn-delta",
        type=int,
        default=None,
        help="Left/right speed split used for line-follow turns",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=1.0,
        help="Seconds between live status prints (0 disables periodic status)",
    )
    return parser.parse_args()


def apply_args_to_config(args: argparse.Namespace, cfg: MissionConfig) -> None:
    cfg.obstacle.trigger_distance_cm = args.obstacle_cm
    cfg.ball.approach_stop_distance_cm = args.pickup_cm
    cfg.home.drop_distance_cm = args.home_drop_cm
    if args.calibration_samples is not None:
        cfg.home.calibration_samples = max(1, args.calibration_samples)
    if args.cycle_sleep is not None:
        cfg.cycle_sleep_s = max(0.01, args.cycle_sleep)
    if args.line_base_speed is not None:
        cfg.line.base_speed = max(200, args.line_base_speed)
    if args.line_turn_delta is not None:
        cfg.line.turn_speed_delta = max(50, args.line_turn_delta)


def read_runtime_command() -> Optional[str]:
    """Read one non-blocking command from stdin when running interactively."""
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
    return line.strip().lower()


def main() -> None:
    args = parse_args()
    cfg = MissionConfig()
    apply_args_to_config(args, cfg)

    adapter = CarAdapter(config=cfg)
    mission = LineAvoidBallHomeMission(adapter=adapter, config=cfg)
    status_interval_s = max(0.0, args.status_interval)
    last_status_ts = 0.0

    print("[challenge] starting entrypoint")
    print(
        "[challenge] obstacle_cm=%.1f pickup_cm=%.1f home_drop_cm=%.1f"
        % (
            cfg.obstacle.trigger_distance_cm,
            cfg.ball.approach_stop_distance_cm,
            cfg.home.drop_distance_cm,
        )
    )
    if status_interval_s > 0:
        print(f"[challenge] status updates enabled ({status_interval_s:.1f}s)")
    else:
        print("[challenge] status updates disabled")

    adapter.start()
    try:
        if args.skip_calibration:
            print("[challenge] home marker calibration skipped")
        else:
            print(
                "[challenge] Place home marker at camera center, then press Enter to calibrate"
            )
            try:
                input()
            except EOFError:
                print(
                    "[challenge] no stdin available, continuing with automatic calibration"
                )

            calibrated = adapter.calibrate_home_marker(cfg.home.calibration_samples)
            if calibrated:
                print("[challenge] home marker calibrated")
            else:
                print(
                    "[challenge] home marker calibration failed; fallback search remains active"
                )

        print("[challenge] runtime commands: home, status, help")

        while True:
            command = read_runtime_command()
            if command:
                if command == "home":
                    print("[challenge] runtime home calibration requested")
                    calibrated = adapter.calibrate_home_marker(
                        cfg.home.calibration_samples
                    )
                    if calibrated:
                        print("[challenge] home marker calibrated")
                    else:
                        print("[challenge] home marker calibration failed")
                elif command == "status":
                    print(
                        "[challenge] state=%s marker_ready=%s"
                        % (mission.state.value, adapter.is_home_marker_ready())
                    )
                elif command in ("help", "?"):
                    print("[challenge] runtime commands: home, status, help")

            mission.run_once()

            now = time.monotonic()
            if status_interval_s > 0 and now - last_status_ts >= status_interval_s:
                sensors = adapter.read_sensors()
                print(
                    "[challenge][status] state=%s ir=%s distance_cm=%.1f ball=%s marker=%s carrying=%s"
                    % (
                        mission.state.value,
                        sensors.infrared_code,
                        sensors.distance_cm,
                        int(sensors.ball_visible),
                        int(sensors.marker_visible),
                        int(getattr(mission, "_carrying_ball", False)),
                    )
                )
                last_status_ts = now

            time.sleep(cfg.cycle_sleep_s)
    except KeyboardInterrupt:
        print("[challenge] stopping entrypoint")
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()
