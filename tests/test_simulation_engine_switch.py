"""Smoke-test Simulation.from_json engine selection without changing scenario.json."""
import json
import tempfile
from pathlib import Path
from simulation import Simulation

source = Path('scenario.json').resolve()
config = json.loads(source.read_text(encoding='utf-8'))
config['map'] = str((source.parent / config['map']).resolve())
config['telemetry'] = None
config['spawning'] = None
config['fleet'] = {'configs': ['Light']}

with tempfile.TemporaryDirectory() as directory:
    for engine in ('legacy', 'v2'):
        config['mission_engine'] = engine
        path = Path(directory) / 'scenario.json'
        path.write_text(json.dumps(config), encoding='utf-8')
        sim = Simulation.from_json(path)
        assert sim.mission_engine == engine
        assert len(sim.vehicles) == 1
        sim.start()
        for _ in range(10):
            sim.advance(0.1)
        assert abs(sim.time - 1.0) < 1e-9
        if engine == 'v2':
            assert abs(sim.mission_manager.time - sim.time) < 1e-9
            # Idle grounded vehicle must not advance its physics clock.
            assert sim.vehicles[0].sim_time == 0.0
        sim.stop()
        print(f'PASS: {engine} engine initializes and advances correctly')
