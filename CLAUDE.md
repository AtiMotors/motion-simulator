# 3-point-turn

Unicycle-model 3-segment turn simulator in Python. Visualizes a parameterized forward/reverse/forward maneuver with matplotlib (static plot + animation).

## Entry points

- `simulate-3-point-turn.py` — the actual simulator. Run with `uv run simulate-3-point-turn.py`. Note the hyphenated filename: invoke by path, not as a module.
- `main.py` — stub (`uv run main.py`), not the simulator.

## Config

`config.toml` is read at runtime. Keys: `ROBOT_WIDTH`, `ROBOT_LENGTH`, `alpha`/`beta`/`gamma` (heading changes, deg), `d1`/`d2`/`d3` (segment distances), `v` (speed magnitude), `dt` (sim timestep). Missing keys fall back to `DEFAULT_CONFIG` in the script.

The three segments are hardcoded: forward+left, reverse+right, forward+left. To change the maneuver shape, edit `specs` in `main()`; `config.toml` only tunes magnitudes.

## Project setup

- Python >=3.11, managed by `uv` (see `uv.lock`, `.python-version`).
- `pyproject.toml` currently declares **no dependencies** despite the script importing `numpy` and `matplotlib`. If you touch dependencies, add them here rather than relying on the ambient `.venv`.
