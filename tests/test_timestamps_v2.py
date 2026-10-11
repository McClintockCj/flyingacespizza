
"""Run alongside the existing V2 regression suite. Requires project dependencies."""

from test_consecutive_missions_v2 import manager

missions = manager.completed_missions
assert len(missions) == 2, missions

# Validate events within each mission.
for mission in missions:
    events = mission["events"]
    times = [e["time"] for e in events]

    # All timestamps are nonnegative and chronological.
    assert all(t >= 0.0 for t in times)
    assert times == sorted(times), (mission["id"], times)

    starts = [e for e in events if e["type"] == "mission_started"]
    completions = [e for e in events if e["type"] == "mission_completed"]

    assert len(starts) == 1, (mission["id"], starts)
    assert len(completions) == 1, (mission["id"], completions)

    # Mission-relative time resets at flight start.
    assert starts[0]["mission_elapsed_time"] == 0.0

    # Ground operations may precede mission_started.
    assert completions[0]["time"] > starts[0]["time"]

    # Mission elapsed times are nonnegative when present.
    for event in events:
        if "mission_elapsed_time" in event:
            assert event["mission_elapsed_time"] >= 0.0, event

# Validate the transition between missions.
first_completion = next(
    e for e in missions[0]["events"]
    if e["type"] == "mission_completed"
)

second_start = next(
    e for e in missions[1]["events"]
    if e["type"] == "mission_started"
)

assert second_start["time"] > first_completion["time"]

# Validate the manager's global event timeline.
global_times = [e["time"] for e in manager.events]

assert global_times == sorted(global_times), manager.events

print("PASS: mission and ground events share a monotonic simulation clock")
