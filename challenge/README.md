# Challenge Setup

This folder contains a setup scaffold for an autonomous mission:

1. Follow black line.
2. Detect foam ball targets.
3. Pick ball with clamp.
4. Carry and drop ball in a target zone.

This is intentionally a setup, not a full autonomous implementation yet.

## Files

- `config.py`: mission parameters and thresholds.
- `interfaces.py`: adapters over existing `Code/Server` modules.
- `mission.py`: mission state machine skeleton.
- `run_setup.py`: startup runner and dry-run diagnostics.
- `program_base.py`: abstract interface for runnable programs.
- `programs_impl.py`: concrete program implementations.
- `program_registry.py`: named program catalog.
- `program_runner.py`: generic runner for named programs.

## Goal

Provide a clean place to incrementally implement autonomy without breaking current `Code/Server/main.py` behavior.

## New Autonomous Entrypoint

Run:

```bash
python -m challenge.entrypoint
```

Or from inside this folder:

```bash
python3 entrypoint.py
```

Default behavior:

1. Follow line continuously (uses legacy `Code/Server/car.py` line-follow step by default).
2. Avoid obstacle when sonar distance is <= 15 cm.
3. Detect and pick red ball (pickup stop distance defaults to 15 cm).
4. Return to startup home anchor using dead-reckoning geolocation.
5. Drop ball and continue loop until interrupted (`Ctrl-C`).
6. If line is not detected, crawl forward slowly instead of stopping.

Home anchor is set at startup from current robot pose and can be reset at runtime.

Useful options:

```bash
python -m challenge.entrypoint --obstacle-cm 15 --pickup-cm 15 --home-radius-m 0.22
python -m challenge.entrypoint --line-base-speed 1100 --line-turn-delta 140
python -m challenge.entrypoint --ir-zero-lost
python -m challenge.entrypoint --normal-vision-steering
python -m challenge.entrypoint --status-interval 0.5
```

Equivalent direct-file options (when in this folder):

```bash
python3 entrypoint.py --obstacle-cm 15 --pickup-cm 15 --home-radius-m 0.22
python3 entrypoint.py --line-base-speed 1100 --line-turn-delta 140
python3 entrypoint.py --ir-zero-lost
python3 entrypoint.py --normal-vision-steering
python3 entrypoint.py --status-interval 0.5
```

Runtime commands (interactive terminal):

1. `home`: reset geolocation home anchor to current pose.
2. `status`: print current mission/nav state and map counters.
3. `help`: print runtime command list.

`run_setup.py` also prints a periodic status line every second for basic diagnostics.

## Dependency Behavior

1. Recommended: run `python3 Code/setup.py` once on Raspberry Pi to provision all required dependencies.
2. `run_setup.py` and `entrypoint.py` no longer require OpenCV at import-time.
3. Challenge runtime now auto-attempts dependency install for missing modules (`cv2`, `numpy`, `picamera2`, `libcamera`, `gpiozero`, `pigpio`, `lgpio`, `rpi_hardware_pwm`).
4. Auto-install strategy is apt-first (`python3-*` packages), then pip fallback where needed.
5. If auto-install fails, run `python3 Code/setup.py` and retry.

## Program-oriented architecture

`challenge` now owns reusable logic/classes while executable entry scripts live in `Code/Server/programs/`.

This keeps runtime launch points simple for future web integration without coupling mission logic to web code.
