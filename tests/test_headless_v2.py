"""End-to-end V2 headless run using the real Simulation and OrderSpawner.

Run: python test_headless_v2.py --duration 600 --missions 2
"""
from __future__ import annotations

import argparse
import json
import tempfile
from collections import Counter
from pathlib import Path

from simulation import Simulation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='scenario.json')
    parser.add_argument('--duration', type=float, default=600.0)
    parser.add_argument('--missions', type=int, default=2)
    parser.add_argument('--interval', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.1)
    args = parser.parse_args()
    if args.duration <= 0 or args.missions < 1 or args.interval <= 0 or args.dt <= 0:
        parser.error('duration, missions, interval, and dt must be positive')

    source = Path(args.scenario).resolve()
    config = json.loads(source.read_text(encoding='utf-8'))
    map_path = Path(config.get('map', 'maps/richmond_small_3.json'))
    config['map'] = str((source.parent / map_path).resolve()) if not map_path.is_absolute() else str(map_path)
    config['mission_engine'] = 'v2'
    config['fleet'] = {'configs': ['Fast']}
    config['orders'] = []
    config['spawning'] = {'type': 'interval', 'interval': args.interval, 'max_active_orders': 5, 'max_delivery_radius': 100.0}
    # Isolate flight/spawning behavior from telemetry serialization during this first test.
    config['telemetry'] = None
    config.setdefault('ground_operations', {'loading_time': 5.0, 'unloading_time': 3.0,
        'recharge_threshold': 30.0, 'recharge_release_threshold': 80.0,
        'emergency_threshold': 15.0})

    with tempfile.TemporaryDirectory() as directory:
        test_path = Path(directory) / 'scenario.json'
        test_path.write_text(json.dumps(config), encoding='utf-8')
        sim = Simulation.from_json(test_path)
        assert sim.mission_engine == 'v2'
        assert len(sim.vehicles) == 1
        sim.start()
        last_report = -1
        while sim.time_seconds < args.duration and len(sim.mission_manager.completed_missions) < args.missions:
            sim.advance(min(args.dt, args.duration - sim.time_seconds))
            # second = int(sim.time_seconds)
            # if second // 30 != last_report and second % 30 == 0:
            #     last_report = second // 30
            #     counts = dict(Counter(o.status for o in sim.scenario.orders))
            #     print(f't={sim.time_seconds:.1f}s orders={counts} completed={len(sim.mission_manager.completed_missions)} '
            #           f'battery={sim.vehicles[0].get_state().battery.remaining_percent():.1f}%')
        sim.stop()

        orders = sim.scenario.orders
        completed = sim.mission_manager.completed_missions
        counts = Counter(o.status for o in orders)
        print('\nHEADLESS V2 SUMMARY')
        print(f'Simulation time: {sim.time_seconds:.1f}s')
        print(f'Orders spawned: {len(orders)}; status counts: {dict(counts)}')
        print(f'Missions completed: {len(completed)}')
        print(f'Final battery: {sim.vehicles[0].get_state().battery.remaining_percent():.2f}%')
        for mission in completed:
            print(f"  {mission['id']}: status={mission['status']} delivered={mission['delivered_orders']}")
        for event in sim.mission_manager.events:
            if event['type'] in ('loading_started', 'mission_started', 'order_delivered',
                                 'mission_completed', 'recharging_started', 'recharging_completed'):
                print(f"  t={event['time']:.1f} {event['type']} {event.get('mission_id', '')}")

        assert len(orders) > 0, 'Spawner produced no orders'
        assert len(completed) >= args.missions, (
            f'Only {len(completed)} missions completed by {args.duration}s; '
            f'orders={dict(counts)}. Check order feasibility, routing, and dispatch.')
        assert any(o.status == 'delivered' for o in orders), 'No delivered orders'
        assert all(a['time'] <= b['time'] for a, b in zip(sim.mission_manager.events, sim.mission_manager.events[1:])), 'Events out of order'
        print('PASS: V2 headless spawning, dispatch, and mission completion')


if __name__ == '__main__':
    main()
