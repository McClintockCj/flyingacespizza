# Flying Aces Pizza Delivery Simulator

A scenario-driven UAV pizza delivery simulator for exploring **systems engineering**, autonomous fleet operations, mission planning, energy constraints, and vehicle behavior. It combines a real-world map, high-level delivery operations, and multirotor simulation through [RotorPy](https://github.com/ethz-asl/rotorpy).

The project is designed for learning and experimentation: change fleet configurations, demand, wind, and operating conditions; observe the resulting missions; and analyze telemetry from interactive or headless runs. It is a simulation and teaching tool, **not flight-ready autonomy software**.

## What it does

- Simulates a fleet of drones with distinct configurations, names, payload capacities, cruise speeds, and batteries.
- Generates orders procedurally and allows interactive order placement.
- Groups eligible orders into delivery missions, plans obstacle-aware routes, and dispatches available vehicles.
- Executes successive flight legs, including delivery stops and a return to each vehicle's home location, using RotorPy dynamics and control.
- Models battery discharge, charging, payload changes, and mission energy-feasibility checks.
- Displays maps, vehicles, routes, orders, flight status, and battery information in a Pygame interface.
- Supports headless experiments and exports vehicle, mission, and order telemetry.
- Provides scenario editing and save/load functionality, with editor expansion under development.

**Current limitations:** Manual abort and return-to-base behavior are still being refined. Battery and wind-energy models are empirical approximations rather than calibrated aircraft power models. The scenario editor is undergoing redesign; no-fly-zone and preferred-corridor authoring should not yet be treated as completed features.

## Requirements

- **Python 3.13** (use the project's tested version rather than assuming newer Python releases are compatible)
- `pip` and `venv`
- Dependencies from the repository's `requirements.txt`, including Pygame, NumPy, and RotorPy where specified
- A scenario JSON file and referenced map JSON assets

## Installation

Clone the repository and change into its root directory (the directory containing `main.py` and `requirements.txt`).

```bash
git clone <repository-url>
cd <repository-directory>
```

Create a Python 3.13 virtual environment:

**Windows (PowerShell):**

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

**Windows (Command Prompt):**

```bat
py -3.13 -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

**Linux / macOS:**

```bash
python3.13 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Verify your interpreter and dependencies:

```bash
python --version
python -m pip check
```

> If activation is blocked by your shell policy, run the environment's Python directly (`.venv\Scripts\python.exe` on Windows or `.venv/bin/python` on Linux/macOS). Do not install project dependencies into an unrelated Python environment.

## Running

From the repository root:

```bash
python main.py scenario.json
```

Run an experiment without opening the UI:

```bash
python main.py scenario.json --headless --duration 120
```

The duration is measured in **simulated seconds**, not wall-clock seconds. Depending on the CLI version, additional arguments may include `--missions` for a mission-based termination condition and `--output` for telemetry output:

```bash
python main.py --help
```

Use `--help` as the definitive source for the installed version's options. Keep `scenario.json` and the `maps/` resources referenced by the scenario at their expected paths.

## Module overview

The package layout following the reorganization is:

```text
project-root/
├── main.py                       # Thin entry point
├── scenario.json                 # Scenario configuration (project asset)
├── maps/                         # Generated map JSON files (project assets)
├── requirements.txt              # Python dependencies
└── flying_aces/
    ├── cli.py                    # CLI argument parsing and run modes
    ├── simulation.py             # Main simulation orchestration
    ├── telemetry.py              # Telemetry collection and export
    ├── core/
    │   └── contracts.py          # Shared types, states, and interfaces
    ├── aircraft/                 # Drone configs, fleet, battery, trajectory,
    │                             # and RotorPy vehicle adapter
    ├── missions/                 # Mission planner, executor, manager,
    │                             # and energy feasibility
    ├── world/                    # Scenario manager, map manager,
    │                             # and order spawning
    └── ui/                       # Simulator UI, camera, and scenario editor
```

The exact filenames inside each subpackage should be checked against your branch. In particular, the map component was renamed to **MapManager** and the interface to **SimulatorUI** after the initial package reorganization.

### Responsibilities

| Subsystem | Responsibility |
| --- | --- |
| Core contracts | Shared domain objects and interfaces: positions, orders, wind, state, and routes |
| Aircraft | Drone configuration, RotorPy integration, trajectory tracking, battery accounting, and fleet construction |
| Missions | Order batching, route planning, landing legs, delivery execution, mission state transitions, and energy checks |
| World | Loading scenario assets, map geometry, obstacle information, environmental conditions, and order spawning |
| Simulation | Coordinating time advancement, mission operations, world events, and telemetry |
| UI | Rendering the map and operational state, processing user input, and editing scenarios |
| Telemetry | Recording aircraft and mission information for inspection and repeatable experiments |
| CLI | Starting interactive or headless runs with experiment arguments |

### How the systems work together

```text
Scenario + map + fleet + orders + wind
                  |
                  v
           Simulation loop
                  |
           Mission manager
           /      |       \
      Planner  Executor  Energy policy
                  |
          RotorPy adapter
            /          \
       Dynamics      Battery
                  |
            State + events
             /         \
       Pygame UI    Telemetry
```

**Planning** decides what each aircraft is supposed to accomplish; **execution** manages delivery-leg progression; **RotorPy** advances its physical vehicle state. The UI observes and controls the simulation rather than serving as its execution engine.

## Interactive controls

Controls depend on whether the simulator is in simulation or editor mode. The following describes the principal controls from the project's recent UI versions; confirm the final key bindings in `flying_aces/ui/simulator_ui.py`.

| Control | Function |
| --- | --- |
| `H` | Toggle on-screen help |
| `Tab` | Switch between simulation and editor modes |
| `A` | Toggle automatic mission dispatch |
| `Space` | Pause/resume simulation |
| `P` | Plan or preview a mission (version-dependent) |
| `S` | Start/dispatch a mission (version-dependent) |
| `R` | Request return to base; behavior under refinement |
| `Esc` | Abort an active mission; behavior under refinement |
| `N` | Reset simulation/scenario state |
| `Up` / `Down` | Adjust automatic order-spawning interval |
| Mouse wheel | Zoom map |
| Middle-button drag | Pan map |
| Left click | Place an order or apply the selected editor tool, depending on mode |
| `Ctrl+S` / `Ctrl+L` | Save/load scenario where enabled |

Use the built-in `H` help display when running an interactive session. The editor is being redesigned to emphasize **home bases, wind, no-fly zones, and optionally preferred flight corridors**, rather than manually placing building obstacles already imported from map data.

## Maps and scenarios

Maps are represented as JSON and use a local metric coordinate system suitable for simulation. Geographic data can describe buildings, roads, green space, and water; building geometry is also used for obstacle-aware routing. Scenarios provide a controlled way to configure a world, its fleet, and operating conditions.

Keep generated maps separate from scenario-specific controls and orders. This allows the same geography to support many experiments without modifying the underlying map data.

The project has also used standalone OpenStreetMap processing tools to produce map JSON. Those tools may live outside the main `flying_aces` package; consult the map pipeline in your checked-out repository before using it.

## Batteries and energy feasibility

Battery capacity and remaining charge are expressed in **watt-hours (Wh)**. Discharge models estimate electrical power in **watts (W)**, and energy consumption is computed as:

```text
energy_wh = power_w * elapsed_seconds / 3600
```

Available models include constant-power discharge, payload/wind-sensitive discharge, and constant-power recharge. Mission feasibility evaluates the planned flight legs, estimated energy, a safety multiplier, and a reserve requirement before dispatching work. Directional wind adjustments distinguish headwind, crosswind, and tailwind in the newer estimator.

These are **simulation policy models**. They are useful for comparative experiments, but have not been calibrated against measured multirotor electrical consumption. In particular, trajectory-duration estimates may omit extra flight time due to settling, hover, or control response. Predicted and actual mission energy should therefore be compared using telemetry.

## Telemetry and experiments

Telemetry supports inspection of fleet activity and comparative experiments. Depending on configuration, the output can include vehicle positions and velocities, battery energy and percentage, order states, mission execution events, completed mission records, and faults.

Good introductory experiments include varying payload, wind, order arrival rate, fleet composition, and battery capacity while holding other scenario variables constant. Collect and compare deliveries completed, energy consumed, mission duration, failed missions, and recharge downtime.

For reproducibility, record the scenario file, software revision, random seed (when supported), and run arguments with experimental results.

## Development priorities

1. Make normal RTB, manual abort, and emergency recovery clearly defined and testable.
2. Compare actual versus estimated mission energy and include hover/settling allowances.
3. Redesign scenario editing around home locations, wind, no-fly zones, and optional corridors.
4. Add automated tests for mission lifecycle, energy conservation, payload changes, and order requeueing.
5. Expand experiment reporting and reproducibility controls.

## Scope

Flying Aces is intended to support systems engineering education, simulation-driven experimentation, and prototyping of UAV delivery logistics. It does not model every flight-safety consideration and must not be used as an operational flight-control or airworthiness system.
