# Programs

This directory contains standalone scripts that launch named robot programs.

These scripts are intentionally decoupled from any web server implementation.
A future branch can call them (or import the same runner APIs) from HTTP endpoints.

## Current scripts

- `run_line_ball_setup.py`: starts the preliminary line-follow + ball mission setup.

## Run

From repository root:

```powershell
python Code/Server/programs/run_line_ball_setup.py
```
