from __future__ import annotations

import argparse

from flying_aces.ui.simulator_ui import SimulatorUI
from flying_aces.simulation import Simulation


def run_interactive(scenario_path: str) -> None:
    simulation = Simulation.from_json(scenario_path)
    ui = SimulatorUI(
        scenario_manager=simulation.environment,
        mission_manager=simulation.mission_manager,
        simulation=simulation,
    )
    ui.run()


def run_headless(
    scenario_path: str,
    missions: int | None = None,
    duration: float | None = None,
    output: str | None = None,
) -> None:
    simulation = Simulation.from_json(scenario_path)

    if output is not None and simulation.telemetry is not None:
        simulation.telemetry.config.output = output

    print(f"Loaded scenario: {scenario_path}")
    print(f"Fleet size: {len(simulation.vehicles)}")
    print(f"Orders: {len(simulation.scenario.orders)}")
    print()

    dt = 0.1
    simulation.start()

    while simulation.running:
        if missions is not None:
            terminal_missions = (
                len(simulation.mission_manager.completed_missions)
                # + len(simulation.mission_manager.terminated_missions)
            )

            if terminal_missions >= missions:
                break

        if duration is not None and simulation.time_seconds >= duration:
            break

        simulation.advance(dt)

    simulation.stop()

    terminal_missions = (
        len(simulation.mission_manager.completed_missions)
        # + len(simulation.mission_manager.terminated_missions)
    )

    print("Headless simulation complete.")
    print(f"Simulation time: {simulation.time_seconds:.1f}s")
    print(f"Terminal missions: {terminal_missions}")

    if simulation.telemetry is not None:
        print(f"Telemetry written to: {simulation.telemetry.config.output}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pizza Drone Simulation"
    )

    parser.add_argument(
        "scenario",
        help="Path to the simulation scenario JSON file",
    )

    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without the Pygame visualization",
    )

    parser.add_argument(
        "--missions",
        type=int,
        help="Stop after this many missions reach a terminal state",
    )

    parser.add_argument(
        "--duration",
        type=float,
        help="Stop after this many simulation seconds",
    )

    parser.add_argument(
        "--output",
        help="Override the telemetry output path",
    )

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.headless:
        if args.missions is None and args.duration is None:
            parser.error(
                "--headless requires --missions or --duration"
            )

        if args.missions is not None and args.missions < 1:
            parser.error("--missions must be at least 1")

        if args.duration is not None and args.duration <= 0:
            parser.error("--duration must be greater than 0")

        run_headless(
            args.scenario,
            missions=args.missions,
            duration=args.duration,
            output=args.output,
        )
    else:
        run_interactive(args.scenario)


if __name__ == "__main__":
    main()