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

## Goal

Provide a clean place to incrementally implement autonomy without breaking current `Code/Server/main.py` behavior.
