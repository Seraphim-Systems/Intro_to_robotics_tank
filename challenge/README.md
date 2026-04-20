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

1. Follow line using infrared sensors.
2. Avoid obstacle when sonar distance is <= 15 cm.
3. Detect and pick red ball (pickup stop distance defaults to 8 cm).
4. Return to runtime-calibrated home marker.
5. Drop ball and continue loop until interrupted (`Ctrl-C`).

At startup, script prompts for home marker calibration from camera center ROI.

Useful options:

```bash
python -m challenge.entrypoint --obstacle-cm 15 --pickup-cm 8 --home-drop-cm 10
python -m challenge.entrypoint --calibration-samples 20
python -m challenge.entrypoint --skip-calibration
```

Equivalent direct-file options (when in this folder):

```bash
python3 entrypoint.py --obstacle-cm 15 --pickup-cm 8 --home-drop-cm 10
python3 entrypoint.py --calibration-samples 20
python3 entrypoint.py --skip-calibration
```

Runtime commands (interactive terminal):

1. `home`: recalibrate home marker from current camera center view.
2. `status`: print current mission state and marker calibration status.
3. `help`: print runtime command list.

## Dependency Behavior

1. Recommended: run `python3 Code/setup.py` once on Raspberry Pi to provision all required dependencies.
2. `run_setup.py` and `entrypoint.py` no longer require OpenCV at import-time.
3. `grand_factory_autonomous` will auto-attempt pip install for missing `cv2` and `numpy` when launched.
4. If auto-install fails, run `python3 Code/setup.py` and retry.

## Program-oriented architecture

`challenge` now owns reusable logic/classes while executable entry scripts live in `Code/Server/programs/`.

This keeps runtime launch points simple for future web integration without coupling mission logic to web code.
