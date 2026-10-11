"""Headless V2 fleet integration test; leaves production code unchanged.

Run: python test_fleet_headless_v2.py --duration 1200 --missions 3
"""
from __future__ import annotations

import argparse
import json
import tempfile
import time
from collections import Counter
from pathlib import Path

from simulation import Simulation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='scenario.json')
    parser.add_argument('--duration', type=float, default=1200.0)
    parser.add_argument('--missions', type=int, default=3)
    parser.add_argument('--radius', type=float, default=250.0)
    parser.add_argument('--interval', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.1)
    args = parser.parse_args()
    if min(args.duration, args.missions, args.radius, args.interval, args.dt) <= 0:
        parser.error('All numeric arguments must be positive')

    source = Path(args.scenario).resolve()
    config = json.loads(source.read_text(encoding='utf-8'))
    map_path = Path(config.get('map', 'maps/richmond_small_3.json'))
    config['map'] = str((source.parent / map_path).resolve()) if not map_path.is_absolute() else str(map_path)
    config['mission_engine'] = 'v2'
    config['fleet'] = {'configs': ['Light', 'Standard', 'Heavy', 'Fast', 'Endurance']}
    config['orders'] = []
    config['spawning'] = {'type': 'interval', 'interval': args.interval,
                          'max_active_orders': 15, 'max_delivery_radius': args.radius}
    config['telemetry'] = None  # Test fleet dispatch independently of telemetry.
    config.setdefault('ground_operations', {'loading_time': 5.0, 'unloading_time': 3.0,
        'recharge_threshold': 30.0, 'recharge_release_threshold': 80.0,
        'emergency_threshold': 15.0})

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'scenario.json'
        path.write_text(json.dumps(config), encoding='utf-8')
        sim = Simulation.from_json(path)
        assert sim.mission_engine == 'v2'
        assert len(sim.vehicles) == 5
        assert sim.order_spawner.max_delivery_radius == args.radius, 'Spawner radius not configured'
        manager = sim.mission_manager
        sim.start()
        started_wall = time.perf_counter()
        last_report = -1
        ever_active = set()
        observed_overlap = False
        try:
            while sim.time_seconds < args.duration and len(manager.completed_missions) < args.missions:
                sim.advance(min(args.dt, args.duration - sim.time_seconds))
                active = [f for f in manager.flights if f.executor is not None]
                ever_active.update(f.vehicle.id for f in active)
                observed_overlap |= len(active) >= 2

                # A single order must never be assigned to two simultaneous missions.
                assigned = [o.id for f in active for o in (f.orders or [])]
                assert len(assigned) == len(set(assigned)), f'Duplicate active assignment: {assigned}'

                # Print every 60 simulated seconds, not every physics step.
                bucket = int(sim.time_seconds // 60)
                if bucket != last_report:
                    last_report = bucket
                    counts = dict(Counter(o.status for o in sim.scenario.orders))
                    batteries = ', '.join(f'{v.id}={v.get_state().battery.remaining_percent():.0f}%'
                                          for v in sim.vehicles)
                    print(f't={sim.time_seconds:7.1f}s completed={len(manager.completed_missions)} '
                          f'active={len(active)} orders={counts} batteries=[{batteries}]', flush=True)
        finally:
            sim.stop()

        elapsed = time.perf_counter() - started_wall
        completed = manager.completed_missions
        print('\nFLEET V2 HEADLESS SUMMARY')
        print(f'Simulation time: {sim.time_seconds:.1f}s; wall time: {elapsed:.1f}s; '
              f'speed: {sim.time_seconds / elapsed:.2f}x')
        print(f'Orders: {dict(Counter(o.status for o in sim.scenario.orders))}')
        print(f'Completed missions: {len(completed)}; vehicles active at some point: {sorted(ever_active)}')
        print(f'Overlapping missions observed: {observed_overlap}')
        for v in sim.vehicles:
            print(f'  {v.id}: battery={v.get_state().battery.remaining_percent():.2f}%')
        for mission in completed:
            print(f"  {mission['id']}: {mission['status']}; delivered={mission['delivered_orders']}")
        for event in manager.events:
            if event['type'] in ('mission_started', 'mission_completed', 'recharging_started', 'recharging_completed'):
                print(f"  t={event['time']:.1f} {event['vehicle_id']} {event['type']}")

        assert len(sim.scenario.orders) > 0, 'No orders spawned'
        assert len(ever_active) >= 2, 'Fewer than two drones were assigned missions'
        assert observed_overlap, 'No concurrent active missions observed'
        assert len(completed) >= args.missions, f'Only {len(completed)} missions completed; inspect summary'
        assert any(o.status == 'delivered' for o in sim.scenario.orders), 'No orders delivered'
        assert all(a['time'] <= b['time'] for a, b in zip(manager.events, manager.events[1:])), 'Event timestamps out of order'
        print('PASS: V2 fleet headless spawning, concurrent dispatch, and mission completion')


if __name__ == '__main__':
    main()
