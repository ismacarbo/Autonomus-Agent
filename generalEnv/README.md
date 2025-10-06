# Pygame Planner Demo

Minimal quintic planner + pure-pursuit tracker with a kinematic bicycle model, rendered in **pygame**.

## Install & run
```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

## Controls
- `R`: reset to a new random goal.
- `SPACE`: pause/resume simulation.
- `ESC` or window close: quit.

## Notes
- World units are meters; the renderer applies a simple pixels-per-meter scale.
- Planner generates a 5th-order polynomial (quintic) path from start to goal over T seconds.
- Pure-pursuit provides curvature steering; longitudinal control is a simple P controller on speed.
