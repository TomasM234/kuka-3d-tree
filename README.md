<p align="center">
  <img src="DATA/PNG/LogoSocial.png" alt="KUKA.T.R.E.E. Logo" width="400">
</p>

# 🤖 KUKA 3D Print Toolkit

> A set of tools for trajectory import, simulation, and streaming to a KUKA robot.
> Built for people who know what they're doing. Read the disclaimer at the bottom. Seriously.

**Documentation status:** March 2026

---

## What This Is

This repository is a foundation for robot-based 3D printing with a KUKA arm. It is split into two main parts:

- **`ROOT/` — Python toolchain (universal):** trajectory import from multiple formats, conversion to a unified CSV, 3D viewer, reachability simulation, and export via postprocessors.
- **`LEAF/` — KRL programs (KUKA-specific):** streaming trajectory points from a file via the SPS background task, ring-buffered into a continuous robot motion — no fixed trajectory size limit.

---

## Repository Structure

```
├── ROOT/                         Python toolchain
│   ├── csv_viewer_pyvista.py     Main GUI viewer (launch this)
│   ├── robot_sim.py              URDF + IK/FK layer
│   ├── gcode_to_csv.py           Slicer G-code → CSV
│   ├── nc_to_csv.py              NC G-code (M3/M5/S) → CSV
│   ├── gcode_conversion_spec.md  Conversion format specification
│   ├── test_urdf.py              Quick URDF/IK sanity check
│   ├── splash.png                Splash screen on startup
│   ├── viewer_settings.json      Auto-saved viewer settings
│   ├── Postprocesor/             Export postprocessors
│   ├── CSV/                      Example CSV data
│   ├── GCODE/                    Example G-code input files
│   ├── Output/                   Postprocessor output (preview.txt etc.)
│   └── kuka_kr16_support/        URDF + STL meshes for KR16
│
├── LEAF/                         KRL programs — deploy to robot controller
│   ├── sps.sub                   SPS background task (file streaming + speed output)
│   ├── cnc.dat                   Global structs and shared data (ring buffers)
│   ├── cnc.src                   CNC library: Init, CncOpen, CncClose, CncGetNextPoint,
│   │                               MoveLin, MoveLinExact, trigger callback
│   ├── coin2.src                 Example main print program
│   ├── coin2.dat                 Point/frame data for coin2.src (HOME, CPDAT, FDAT)
│   └── $config.dat               I/O signal configuration (ANOUT, OUT mappings)
│
└── DATA/                         Reference files
    ├── STL/                      Part STL files
    └── PNG/                      Images
```

---

## What It Can Do 🚀

### 1) Python Toolchain

- **G-code import:** convert from slicer G-code (`gcode_to_csv.py`) or NC G-code with M3/M5/S spindle control (`nc_to_csv.py`)
- **Auto-detection** of input format (`.gcode`, `.gco`, `.nc`) directly in the viewer — by content analysis, not just extension
- **3D trajectory visualization** (`csv_viewer_pyvista.py`) with fast PyVista rendering:
  - Print moves rendered as tube geometry
  - Travel / retract as thin lines
  - Color coding: green = print, grey = travel, red = retract
- **Point-by-point and layer-by-layer sliders** — synchronized, with debounce for smooth scrubbing
- **Print time estimate** from segment length and `TCP_SPEED`
- **URDF robot** loading with live IK tracking of the current trajectory point
- **Workplace setup:**
  - Table (dimensions and XY position)
  - Robot BASE frame (X, Y, Z, A, B, C)
  - TOOL frame (X, Y, Z, A, B, C)
- **Trajectory feasibility test** — parallel IK across CPU cores:
  - Status: OK / joint limit exceeded / wrist singularity (A5 near 0°) / unreachable
  - Results shown as a color strip under the viewport
- **Trajectory editing:**
  - Shift X / Y (spinbox + increment buttons: ±1 mm, ±100 mm)
  - Rotate around Z through the model centroid (±1°, ±5°)
  - Transform applied live in the viewer
  - Save back to CSV
- **Postprocessor export** — select a script from `Postprocesor/`, preview or save output
- **Settings persistence** — BASE/TOOL, last file, postprocessor, table config auto-saved to `viewer_settings.json`

### 2) Postprocessors

Located in `ROOT/Postprocesor/`:

| File | Description |
|---|---|
| `JedenRadekDvanactSloupcu.py` | **Production format:** 1 line = 1 point — use with `coin2.src` + `cnc.src` + `sps.sub` |
| `DvaRadkyOsmSloupcu.py` | **Alternative format:** 2 lines per point (header + data) — for older `cnc2/coin4` branch |
| `1_to_1_export.py` | Test-only: straight copy of CSV to text with CRLF line endings |
| `null.py` | Empty placeholder |

### 3) KRL Programs — `LEAF/`

- **`sps.sub`** — SPS background task:
  - Handles OPEN / CLOSE / RESET commands from the main program
  - Reads the file line by line (`krl_fgets`) and parses into the ring buffer
  - Handshake via `CNC_REQ_ID / CNC_ACK_ID`
  - Outputs extruder speed to PLC (`s_RP_MV_Spd`) as a reinterpreted float — corrected by actual robot velocity (`S_Robot_Vel`)

- **`cnc.dat`** — global structs and shared data:
  - `CNC_BUF_POINTS[100]` — point ring buffer (size 100)
  - `CNC_TRG_BUF[200]` — trigger payload buffer (size 200)

- **`cnc.src`** — CNC library:
  - `Init()` — motion parameter setup (tool, base, advance, velocity, acceleration)
  - `CncOpen(fileName[])` / `CncClose()` — open/close the point stream
  - `CncGetNextPoint(point)` — dequeue next point from ring buffer
  - `MoveLin(point)` — approximated LIN move with `C_DIS`
  - `MoveLinExact(point)` — exact LIN stop
  - `CncTrigEnqueue()` / `CncOnPointStart()` — trigger callback for speed handoff to SPS

- **`coin2.src`** — example main print program:
  - Opens stream from `coin.txt`
  - Pipeline: always fetches the next point ahead to enable `C_DIS` approximation
  - Last point is always executed exactly with `LIN` (no `C_DIS`)
  - Handles unlimited trajectory length — no fixed SRC point count limit
  - Includes hardware I/O setup (temperature requests, enable signals)

- **`coin2.dat`** — positional data for `coin2.src` (HOME, intermediate position P2, CPDAT, FDAT)

- **`$config.dat`** — I/O signal configuration: `S_Robot_Vel`, `s_RP_MV_Spd`, temperature and enable outputs mapped to PLC channels

---

## Universal CSV Format

Delimiter: `;`

```
TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
```

| Column | Meaning |
|---|---|
| `TYPE` | `P` print, `T` travel, `R` retract, `U` unretract |
| `X/Y/Z` | TCP position [mm] |
| `A/B/C` | TCP orientation [deg] |
| `TCP_SPEED` | TCP velocity [mm/s] |
| `E_RATIO` | Material flow (delta_E / distance for G-code; S value for NC) |
| `TEMP` | Extruder temperature setpoint [°C] |
| `FAN_PCT` | Fan percentage [0–100] |
| `LAYER` | Layer index |
| `FEATURE` | Feature type ID (perimeter, infill, support…) |
| `PROGRESS` | Print progress estimate [0–100%] |

---

## Postprocessor → KRL Binding

### For `coin2.src` + `cnc.src` + `sps.sub`

Use **`JedenRadekDvanactSloupcu.py`**. Output format (one line per point):

```
X Y Z A B C V_MOVE V_EXT POINT_NO LAYER_NO PROGRESS NOTE
```

This is what `sps.sub` parses via three sequential `SREAD` calls.

### For the older `cnc2/coin4` branch

Use **`DvaRadkyOsmSloupcu.py`**:
- Line 1: text header
- Line 2: `X Y Z A B C V_MOVE V_EXT`

---

## Quick Start — Python

> Run all commands from inside the `ROOT/` directory.

**1. Install dependencies (minimum):**

```bash
pip install PyQt6 numpy pyvista pyvistaqt ikpy yourdfpy trimesh
```

*(No `requirements.txt` yet — contributions welcome.)*

**2. Convert input to CSV:**

```bash
# Slicer G-code (PrusaSlicer, Bambu, etc.)
python gcode_to_csv.py GCODE/model.gcode -o CSV/model.csv

# NC G-code (M3/M5/S extruder control)
python nc_to_csv.py GCODE/model.nc -o CSV/model.csv
```

**3. Launch the viewer:**

```bash
python csv_viewer_pyvista.py
```

**4. In the viewer:**

1. **General tab** → Load CSV
2. **General tab** → Load URDF (`kuka_kr16_support/urdf/kr16_2.urdf`)
3. **Workplace tab** → Set BASE and TOOL frames to match your cell
4. **General tab** → Run `Test Trajectory` — check the color strip for problems
5. **Edit tab** → Adjust position / rotation if needed, save back to CSV
6. **Export tab** → Select postprocessor → Preview or Export

---

## Quick Start — KRL (Robot Side)

> You need KSS 8.2 or compatible. You need robot programming experience. Don't skip step 5.

**1.** Copy `LEAF/` contents to the controller: `cnc.dat`, `cnc.src`, `coin2.src`, `coin2.dat`, `$config.dat`.

**2.** Generate the point file using the postprocessor (from inside `ROOT/`):

```bash
python Postprocesor/JedenRadekDvanactSloupcu.py CSV/model.csv Output/coin.txt
```

Or use the viewer's **Export tab**.

**3.** Transfer `coin.txt` to the directory referenced in `coin2.src`:

```krl
fileName[] = "coin.txt"
```

By default: `KRC:\R1\3dTisk\coin.txt` (set via `&PARAM DISKPATH`).

**4.** Enable SPS by activating `sps.sub`.

**5.** Verify I/O signal mapping in `$config.dat`:

| Signal | Mapping |
|---|---|
| `S_Robot_Vel` | `$ANOUT[10]` |
| `s_RP_MV_Spd` | `$OUT[328..359]` |
| Temperature / enable signals | per your local PLC wiring |

**6.** Run `coin2.src`. Watch the robot move. Watch the PLC receive the speed signal. Be present. Have E-stop within arm's reach.

---

## Known Limitations

- No real collision detection in the viewer — the trajectory test checks IK feasibility, joint limits, and wrist singularity only. It does **not** detect self-collision or environment collision.
- Orientation columns `A/B/C` from CSV are loaded and exported but the viewer does **not** use them as per-point TCP orientation in the IK follower (XYZ position only).
- `gcode_to_csv.py` handles common slicer G-code but is not a full dialect implementation: no `G2/G3` arc support, no full relative-extrusion state machine beyond what PrusaSlicer emits.
- `nc_to_csv.py` assumes M3/M5 + standalone `S` spindle convention; other NC flavors may need parser edits.
- No automated test suite, no CI pipeline.
- No `requirements.txt` / `pyproject.toml`.
- `coin.txt` is **not** part of this repository — you must generate it with a postprocessor before running the robot program.

---

---

---

# ⚠️ READ THIS BEFORE YOU TOUCH ANYTHING ⚠️

## THIS IS A TOOLKIT. NOT A PRODUCT.

**THIS IS NOT, AND WILL NEVER BE, AN OUT-OF-THE-BOX SOLUTION.**

This repository is a set of tools — carefully engineered, reasonably tested on one specific cell, and deliberately left open for modification. It is a starting point, not a finish line.

**No two robot cells are the same.**

Your tool frame is different. Your base frame is different. Your PLC signal mapping is different. Your extruder is different. Your slicer is different. Your safety zone is different. **You will need to adapt this code to your specific setup before running a single move.**

Using this repository without understanding and modifying it for your cell configuration is not just likely to fail — it may cause your robot to move in unexpected ways, potentially damaging equipment or injuring people.

**⚡ THIS SOFTWARE IS PROVIDED WITHOUT ANY WARRANTY, EXPRESS OR IMPLIED.**

The authors assume no liability for:
- Robot collisions
- Damaged workpieces, fixtures, or robot components
- Incorrect material deposition
- Any other damage, financial loss, or injury resulting from the use of this software

**🦺 THIS IS FOR EXPERIENCED ROBOT OPERATORS ONLY.**

If you are not comfortable with:
- Writing and modifying KRL programs
- Robot cell safety procedures
- Manual jogging and E-stop protocols
- Signal-level PLC/robot interfacing

...then please do not run this software on a real robot until you are.

**Use it. Extend it. Break it. Fix it. Make it yours. Just know what you're doing. 🤙**
