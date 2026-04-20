# Challenge Runtime

This directory is intentionally minimal and uses `Code/Server/car.py` directly.

## Run

From repo root:

```bash
python3 -m challenge.main
```

Or directly:

```bash
python3 challenge/main.py
```

## Mission Behavior

1. Follow black line continuously using `car.mode_infrared()`.
2. If a close object is detected, perform avoid maneuver.
3. If pickup-distance is reached, run clamp pickup sequence.
4. Return to startup pose (home anchor) using dead-reckoning.
5. Drop ball at home and continue.
6. If line is lost, crawl forward slowly.

## Runtime Commands

1. `w a s d` manual pulse movement.
2. `space` manual pick/drop toggle.
3. `home` reset home anchor at current pose.
4. `status` print current mission status.
5. `help` print command list.

## Useful Options

```bash
python3 -m challenge.main --obstacle-cm 18 --pickup-cm 15 --home-radius-m 0.22
python3 -m challenge.main --status-interval 0.5 --loop-sleep 0.05
python3 -m challenge.main --line-crawl-speed 260 --ir-zero-lost
```
