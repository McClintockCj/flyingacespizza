"""Diagnose V2 post-mission dispatch without changing production code.

Run: python test_dispatch_diagnostics_v2.py --duration 900
"""
from __future__ import annotations
import argparse
import json
import tempfile
from pathlib import Path
from collections import Counter
from simulation import Simulation
from contracts import Vector3
from energy_feasibility_v2 import estimate_mission_energy


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', default='scenario.json')
    p.add_argument('--duration', type=float, default=900.0)
    p.add_argument('--dt', type=float, default=0.1)
    args = p.parse_args()
    source = Path(args.scenario).resolve()
    cfg = json.loads(source.read_text(encoding='utf-8'))
    mp = Path(cfg.get('map', 'maps/richmond_small_3.json'))
    cfg['map'] = str((source.parent / mp).resolve() if not mp.is_absolute() else mp)
    cfg.update(mission_engine='v2', fleet={'configs': ['Light']}, orders=[],
               spawning={'type': 'interval', 'interval': 10.0, 'max_active_orders': 5}, telemetry=None)
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / 'scenario.json'
        path.write_text(json.dumps(cfg), encoding='utf-8')
        sim = Simulation.from_json(path)
        sim.start()
        while sim.time_seconds < args.duration and not sim.mission_manager.completed_missions:
            sim.advance(min(args.dt, args.duration - sim.time_seconds))
        mgr = sim.mission_manager
        flight = mgr.flights[0]
        vehicle = flight.vehicle
        state = vehicle.get_state()
        battery = state.battery
        base = sim.scenario.base
        landing_base = Vector3(base.x, base.y, mgr.leg_planner.landing_height)
        print(f'\nTime: {sim.time_seconds:.1f}s; completed missions: {len(mgr.completed_missions)}')
        print(f'Orders: {dict(Counter(o.status for o in sim.scenario.orders))}')
        print(f'Flight state={flight.state}, ground_phase={flight.ground_phase}, executor={flight.executor is not None}')
        print(f'Position={state.position}; distance to landing base={state.position.distance_to(landing_base):.2f}m')
        print(f'Battery={battery.remaining_energy():.2f}/{battery.capacity_wh:.2f} Wh, payload capacity={state.payload_capacity:.2f}kg')
        print(f'Candidate now: {mgr._candidate_mission(flight) is not None}; at full charge: {mgr._candidate_mission(flight, assume_full=True) is not None}')
        print('\nPENDING ORDER DIAGNOSTICS')
        for o in mgr.pending_orders:
            dist = o.destination.distance_to(base)
            if o.payload_weight > state.payload_capacity:
                print(f'{o.id}: {dist:.0f}m from base, {o.payload_weight:.1f}kg: OVER PAYLOAD LIMIT')
                continue
            try:
                mission = mgr.leg_planner.plan_mission(f'diagnostic-{o.id}', state.position, [o], base)
                budget = estimate_mission_energy(vehicle, mission, [o], mgr.environment.get_wind(), mgr.energy_policy)
                required = budget.budgeted_wh + budget.reserve_wh
                verdict = 'FEASIBLE NOW' if required <= battery.remaining_energy() else ('RECHARGE NEEDED' if required <= battery.capacity_wh else 'INFEASIBLE AT FULL CHARGE')
                print(f'{o.id}: {dist:.0f}m from base, {o.payload_weight:.1f}kg, estimated={budget.estimated_wh:.1f}Wh, budgeted={budget.budgeted_wh:.1f}Wh, reserve={budget.reserve_wh:.1f}Wh, required={required:.1f}Wh: {verdict}')
            except (RuntimeError, ValueError) as exc:
                print(f'{o.id}: {dist:.0f}m from base, {o.payload_weight:.1f}kg: PLANNING ERROR: {exc}')
        sim.stop()

if __name__ == '__main__':
    main()
