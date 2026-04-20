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
        default=15.0,
        help="Sonar stop distance in cm before clamp pickup",
    )
    parser.add_argument(
        "--home-radius-m",
        type=float,
        default=0.22,
        help="Geolocation radius in meters that counts as arriving at home drop zone",
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
        "--ir-zero-lost",
        action="store_true",
        help="Treat IR code 0 as line-lost instead of center-line",
    )
    parser.add_argument(
        "--normal-vision-steering",
        action="store_true",
        help="Disable inverted vision steering",
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
    cfg.geo.home_arrival_radius_m = max(0.05, args.home_radius_m)
    if args.cycle_sleep is not None:
        cfg.cycle_sleep_s = max(0.01, args.cycle_sleep)
    if args.line_base_speed is not None:
        cfg.line.base_speed = max(200, args.line_base_speed)
    if args.line_turn_delta is not None:
        cfg.line.turn_speed_delta = max(50, args.line_turn_delta)
    if args.ir_zero_lost:
        cfg.line.code_zero_is_center = False
    if args.normal_vision_steering:
        cfg.ball.invert_steering = False


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
        "[challenge] obstacle_cm=%.1f pickup_cm=%.1f home_radius_m=%.2f"
        % (
            cfg.obstacle.trigger_distance_cm,
            cfg.ball.approach_stop_distance_cm,
            cfg.geo.home_arrival_radius_m,
        )
    )
    print(
        "[challenge] line_base=%d turn_delta=%d ir_zero_center=%s vision_invert=%s"
        % (
            cfg.line.base_speed,
            cfg.line.turn_speed_delta,
            int(cfg.line.code_zero_is_center),
            int(cfg.ball.invert_steering),
        )
    )
    if status_interval_s > 0:
        print(f"[challenge] status updates enabled ({status_interval_s:.1f}s)")
    else:
        print("[challenge] status updates disabled")

    adapter.start()
    try:
        mission.reset_home_anchor()
        print("[challenge] geolocation home anchor set at startup")
        print("[challenge] runtime commands: home, status, help")

        while True:
            command = read_runtime_command()
            if command:
                if command == "home":
                    mission.reset_home_anchor()
                    print("[challenge] geolocation home anchor reset to current pose")
                elif command == "status":
                    nav = mission.get_navigation_status()
                    print(
                        "[challenge] state=%s pos=(%.2f,%.2f)m heading=%.1fdeg home=%.2fm map(lines=%d obstacles=%d)"
                        % (
                            mission.state.value,
                            nav["x_m"],
                            nav["y_m"],
                            nav["heading_deg"],
                            nav["home_distance_m"],
                            nav["line_points"],
                            nav["obstacles"],
                        )
                    )
                elif command in ("help", "?"):
                    print("[challenge] runtime commands: home, status, help")

            mission.run_once()

            now = time.monotonic()
            if status_interval_s > 0 and now - last_status_ts >= status_interval_s:
                sensors = adapter.read_sensors()
                nav = mission.get_navigation_status()
                print(
                    "[challenge][status] state=%s ir=%s distance_cm=%.1f ball=%s carrying=%s home_m=%.2f map_l=%d map_o=%d"
                    % (
                        mission.state.value,
                        sensors.infrared_code,
                        sensors.distance_cm,
                        int(sensors.ball_visible),
                        int(getattr(mission, "_carrying_ball", False)),
                        nav["home_distance_m"],
                        nav["line_points"],
                        nav["obstacles"],
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
