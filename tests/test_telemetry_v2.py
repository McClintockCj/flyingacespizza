"""Headless telemetry integration smoke test; uses existing scenario and RotorPy."""
import json
import tempfile
from pathlib import Path
from simulation import Simulation

source = Path('scenario.json').resolve()
config = json.loads(source.read_text(encoding='utf-8'))
map_path = Path(config['map'])
config['map'] = str((source.parent / map_path).resolve()) if not map_path.is_absolute() else str(map_path)
config['mission_engine'] = 'v2'
config['fleet'] = {'configs': ['Fast']}
config['orders'] = []
config['spawning'] = {'type': 'interval', 'interval': 10.0,
                      'max_active_orders': 5, 'max_delivery_radius': 150.0}
with tempfile.TemporaryDirectory() as tmp:
    config['telemetry'] = {'enabled': True, 'output': str(Path(tmp) / 'telemetry.json'),
                           'state_interval': 5.0, 'vehicles': True, 'orders': True,
                           'missions': True, 'battery': True}
    scenario = Path(tmp) / 'scenario.json'
    scenario.write_text(json.dumps(config), encoding='utf-8')
    sim = Simulation.from_json(scenario)
    sim.start()
    try:
        while sim.time < 1000 and not sim.mission_manager.completed_missions:
            sim.advance(0.1)
    finally:
        sim.stop()
    data = json.loads(Path(config['telemetry']['output']).read_text(encoding='utf-8'))
    assert data['vehicles'] and data['vehicles'][0]['samples']
    assert data['orders']
    assert data['events']
    assert any(e['type'] == 'mission_started' for e in data['events'])
    assert data['missions']
    ids = [m['id'] for m in data['missions']]
    assert len(ids) == len(set(ids)), 'Duplicate mission records'
    assert all(a['time'] <= b['time'] for a,b in zip(data['events'],data['events'][1:])), 'Unordered events'
    print('PASS: V2 telemetry writes vehicle samples, orders, missions, and events')
    print('Time:', round(sim.time,1), 'missions:', len(data['missions']), 'events:', len(data['events']))
