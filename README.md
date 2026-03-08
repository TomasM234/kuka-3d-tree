<p align="center">
  <img src="DATA/PNG/Logo.png" alt="KUKA.T.R.E.E. Logo" width="400">
</p>

# KUKA 3D Print Toolkit

Python tools for trajectory import, simulation, validation, and export, plus
KRL runtime code for streaming point files to a KUKA controller.

Documentation status: March 2026

## Repository Layout

- `ROOT/`
  - `viewer_app.py`: launcher and dependency factory for the desktop viewer
  - `viewer.py`: composed main window class built from viewer mixins
  - `robot_ik.py`: URDF loading, FK, IK, reachability checks
  - `trajectory_schema.py`: shared trajectory parsing, editing, and CSV helpers
  - `Importer/parse_gcode.py`: slicer G-code to universal CSV
  - `Importer/parse_nc.py`: NC G-code with `M3/M5/S` extrusion to universal CSV
  - `Postprocesor/`: production postprocessors used by the viewer export tab
  - `tools/`: development and one-off utility scripts
- `LEAF/`
  - `cnc.dat`: shared data structures and global constants
  - `cnc.src`: stream control and motion helpers
  - `sps.sub`: SPS background reader and ring-buffer feeder
  - `coin2.src`: example robot-side print program

## ROOT Capabilities

- Import slicer G-code and NC G-code into one CSV schema.
- Visualize the trajectory in PyVista.
- Load URDF robots and run point-by-point IK.
- Test the full trajectory in parallel and display status by point.
- Edit the loaded trajectory by planar translation and Z rotation.
- Export robot-ready text output through postprocessor plugins.
- Persist project-specific state in `ROOT/Project/*.json`.

## Universal CSV Format

Delimiter: `;`

```text
TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
```

- `TYPE`: `P`, `T`, `R`, `U`
- `X/Y/Z`: TCP position in mm
- `A/B/C`: TCP orientation in degrees
- `TCP_SPEED`: TCP speed in mm/s
- `E_RATIO`: material flow in g/m
- `TEMP`: extruder temperature setpoint
- `FAN_PCT`: fan percentage
- `LAYER`: layer index
- `FEATURE`: feature identifier
- `PROGRESS`: progress estimate 0-100

## Postprocessors

Production postprocessors live in `ROOT/Postprocesor/`:

- `JedenRadekDvanactSloupcu.py`: one line per point, used by `coin2.src`
- `DvaRadkyOsmSloupcu.py`: legacy two-line export format

Development utilities moved to `ROOT/tools/`:

- `1_to_1_export.py`: raw text copy with CRLF normalization
- `test_seed.py`: IK seed inspection helper

## Quick Start: Python

Run commands from the repository root.

Install dependencies:

```bash
pip install PyQt6 numpy pyvista pyvistaqt ikpy yourdfpy trimesh
```

Convert slicer G-code:

```bash
python -m ROOT.Importer.parse_gcode ROOT/GCODE/model.gcode -o ROOT/CSV/model.csv
```

Convert NC G-code:

```bash
python -m ROOT.Importer.parse_nc ROOT/GCODE/model.nc -o ROOT/CSV/model.csv
```

Launch the viewer:

```bash
python -m ROOT
```

## Quick Start: KRL

1. Export a point file from the viewer or run:

```bash
python -m ROOT.Postprocesor.JedenRadekDvanactSloupcu ROOT/CSV/model.csv ROOT/Output/coin.txt
```

2. Copy `LEAF/cnc.dat`, `LEAF/cnc.src`, `LEAF/sps.sub`, `LEAF/coin2.src`, and related data files to the controller.
3. Transfer `coin.txt` to the path expected by `coin2.src`.
4. Verify I/O mapping in `$config.dat`.
5. Run `sps.sub`, then start `coin2.src`.

## Known Limitations

- The viewer checks IK feasibility, joint limits, and singularities, but not collisions.
- `Importer/parse_gcode.py` does not implement full G-code dialect coverage such as `G2/G3`.
- `Importer/parse_nc.py` assumes the `M3/M5` plus standalone `S` convention.
- There is no packaged dependency manifest yet.
