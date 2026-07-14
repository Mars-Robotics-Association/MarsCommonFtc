#!/usr/bin/env python3
"""Generate golden vectors for RuckigJava differential tests.

Pins: ruckig source tag v0.17.3 (SHA cb99a04). The PyPI sdist for 0.17.3 does not
build with scikit-build-core >= 0.10; build from the vendored tree instead:

  1. Copy the repo's vendored ruckig/ checkout somewhere writable.
  2. In its pyproject.toml change `cmake.targets` -> `build.targets`
     (write the file WITHOUT a UTF-8 BOM).
  3. pip install <that directory>   (needs MSVC/clang/gcc + cmake)

Then:
  python generate_golden.py --out ../src/test/resources/golden

Output: differential_v1.json — directed edge cases plus seeded random batches,
each with the exact input, Result code, duration, and (p, v, a) sampled at 32
times. Infinities are encoded as 1e999 so the Java-side minimal JSON parser
(Double.parseDouble) reads them back as Infinity.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path


def try_import_ruckig():
    try:
        import ruckig
        return ruckig
    except ImportError as e:
        print("ERROR: could not import ruckig (see build notes in this file's docstring).",
              file=sys.stderr)
        print(str(e), file=sys.stderr)
        sys.exit(1)


def sample_times(duration: float, n: int = 32):
    if duration <= 0:
        return [0.0]
    return [duration * i / (n - 1) for i in range(n)]


def result_int(res):
    try:
        return int(res)
    except TypeError:
        return int(res.value)


def input_snapshot(inp):
    snap = {
        "current_position": list(inp.current_position),
        "current_velocity": list(inp.current_velocity),
        "current_acceleration": list(inp.current_acceleration),
        "target_position": list(inp.target_position),
        "target_velocity": list(inp.target_velocity),
        "target_acceleration": list(inp.target_acceleration),
        "max_velocity": list(inp.max_velocity),
        "max_acceleration": list(inp.max_acceleration),
        "max_jerk": list(inp.max_jerk),
    }
    if inp.min_velocity is not None:
        snap["min_velocity"] = list(inp.min_velocity)
    if inp.min_acceleration is not None:
        snap["min_acceleration"] = list(inp.min_acceleration)
    if inp.control_interface.name != "Position":
        snap["control_interface"] = inp.control_interface.name
    if inp.synchronization.name != "Time":
        # Python binding names C++ `None` as `No`; emit the C++/Java name.
        name = inp.synchronization.name
        snap["synchronization"] = "None" if name == "No" else name
    if inp.minimum_duration is not None:
        snap["minimum_duration"] = float(inp.minimum_duration)
    return snap


def case_from_input(inp, case_id: str):
    """One-shot differential case: inputs -> result, duration, sampled kinematics."""
    from ruckig import Ruckig, Trajectory

    dofs = inp.degrees_of_freedom
    case = {"id": case_id, "dofs": dofs, "input": input_snapshot(inp)}
    otg = Ruckig(dofs)
    traj = Trajectory(dofs)
    try:
        res = otg.calculate(inp, traj)
    except Exception as e:
        # Upstream threw (hard-invalid input). Not representable as a Result
        # golden; caller should drop the case.
        print(f"  dropped {case_id}: {e}", file=sys.stderr)
        return None
    case["result"] = result_int(res)
    if case["result"] < 0:
        return case  # error result: nothing to sample

    duration = traj.duration
    case["duration"] = float(duration)
    samples = []
    for t in sample_times(duration):
        p, v, a = traj.at_time(t)
        samples.append({"t": t, "p": list(p), "v": list(v), "a": list(a)})
    case["samples"] = samples
    return case


def fresh_input(dofs: int):
    from ruckig import InputParameter
    inp = InputParameter(dofs)
    inp.current_position = [0.0] * dofs
    inp.current_velocity = [0.0] * dofs
    inp.current_acceleration = [0.0] * dofs
    inp.target_position = [0.0] * dofs
    inp.target_velocity = [0.0] * dofs
    inp.target_acceleration = [0.0] * dofs
    inp.max_velocity = [1.0] * dofs
    inp.max_acceleration = [1.0] * dofs
    inp.max_jerk = [1.0] * dofs
    return inp


def directed_cases():
    from ruckig import ControlInterface, Synchronization
    INF = float("inf")
    cases = []

    def add(case_id, dofs=1, **kw):
        inp = fresh_input(dofs)
        for key, val in kw.items():
            setattr(inp, key, val)
        c = case_from_input(inp, case_id)
        if c is not None:
            cases.append(c)

    # Baseline rest-to-rest (kept from the original smoke suite)
    add("rest_to_rest_1dof", target_position=[1.0],
        max_velocity=[2.0], max_acceleration=[2.0], max_jerk=[4.0])
    add("zero_distance")
    add("tiny_distance", target_position=[1e-6])
    add("negative_direction", target_position=[-3.0])
    add("wrongway_v0", current_velocity=[-0.8], target_position=[2.0])
    add("wrongway_v0_helpful_a0", current_velocity=[-0.8],
        current_acceleration=[0.9], target_position=[2.0])

    # Brake pre-trajectory triggers
    add("brake_v0_over_vmax", current_velocity=[1.5], target_position=[2.0])
    add("brake_v0_far_over_vmax", current_velocity=[3.0], target_position=[0.5])
    add("brake_a0_over_amax", current_acceleration=[1.5], target_position=[2.0])
    add("brake_both_over", current_velocity=[1.8],
        current_acceleration=[-1.6], target_position=[-1.0])

    # Nonzero target states (the capability SCurvePosition lacks)
    add("nonzero_vf", target_position=[2.0], target_velocity=[0.4])
    add("nonzero_vf_af", target_position=[2.0],
        target_velocity=[0.4], target_acceleration=[0.2])
    add("passthrough_fast_vf", target_position=[1.0],
        target_velocity=[0.9], max_velocity=[1.0])

    # Asymmetric (signed-frame) limits
    add("asym_min_velocity", target_position=[-2.0], min_velocity=[-0.3])
    add("asym_min_acceleration", target_position=[2.0],
        min_acceleration=[-0.25], current_velocity=[0.5])
    add("asym_both_min", target_position=[-2.0],
        min_velocity=[-0.4], min_acceleration=[-0.3])

    # Limit scale extremes
    add("tiny_limits", target_position=[0.01],
        max_velocity=[1e-3], max_acceleration=[1e-3], max_jerk=[1e-3])
    add("huge_limits", target_position=[1000.0],
        max_velocity=[1e5], max_acceleration=[1e5], max_jerk=[1e5])
    add("mixed_scale", target_position=[5.0],
        max_velocity=[1e-2], max_acceleration=[10.0], max_jerk=[1e4])

    # Lower-order interfaces via infinite limits
    add("second_order_inf_jerk", target_position=[1.0], max_jerk=[INF])
    add("second_order_inf_jerk_v0", target_position=[1.0],
        current_velocity=[0.5], max_jerk=[INF])
    add("first_order_inf_jerk_accel", target_position=[1.0],
        max_acceleration=[INF], max_jerk=[INF])

    # Minimum duration stretch
    add("minimum_duration", target_position=[0.5], minimum_duration=5.0)

    # Velocity interface
    add("velocity_interface_1dof",
        control_interface=ControlInterface.Velocity,
        target_velocity=[1.0], max_acceleration=[2.0], max_jerk=[4.0])
    add("velocity_interface_nonzero_af",
        control_interface=ControlInterface.Velocity,
        current_velocity=[-0.5], target_velocity=[1.0],
        target_acceleration=[0.3], max_acceleration=[2.0], max_jerk=[4.0])
    add("velocity_interface_brake_a0",
        control_interface=ControlInterface.Velocity,
        current_acceleration=[1.5], target_velocity=[-1.0])
    add("velocity_interface_second_order",
        control_interface=ControlInterface.Velocity,
        target_velocity=[1.0], max_jerk=[INF])

    # Multi-DoF synchronization
    add("sync_time_3dof", dofs=3,
        target_position=[1.0, 0.1, -2.0],
        max_velocity=[1.0, 2.0, 0.5],
        max_acceleration=[1.0, 1.0, 2.0],
        max_jerk=[2.0, 4.0, 8.0])
    add("sync_phase_3dof", dofs=3,
        synchronization=Synchronization.Phase,
        target_position=[1.0, 2.0, -0.5],
        max_velocity=[1.0, 1.0, 1.0],
        max_acceleration=[1.0, 1.0, 1.0],
        max_jerk=[2.0, 2.0, 2.0])
    add("sync_none_2dof", dofs=2,
        synchronization=Synchronization.No,
        target_position=[1.0, 0.01])
    add("sync_time_if_necessary_2dof", dofs=2,
        synchronization=Synchronization.TimeIfNecessary,
        target_position=[1.0, 0.9])
    add("sync_time_blocked_interval_2dof", dofs=2,
        target_position=[1.0, 1.02],
        current_velocity=[0.9, 0.0],
        max_velocity=[1.0, 1.0])

    return cases


def random_cases(seed_label: str, seed: int, count: int, make_input):
    cases = []
    rng = random.Random(seed)
    for i in range(count):
        inp = make_input(rng)
        c = case_from_input(inp, f"{seed_label}_{i:03d}")
        if c is not None:
            cases.append(c)
    return cases


def rand_general(rng):
    """Original smoke regime: states within limits, seed 1 preserved."""
    inp = fresh_input(1)
    inp.current_position = [rng.uniform(-1, 1)]
    inp.current_velocity = [rng.uniform(-1, 1)]
    inp.current_acceleration = [rng.uniform(-1, 1)]
    inp.target_position = [rng.uniform(-1, 1)]
    inp.target_velocity = [rng.uniform(-0.5, 0.5)]
    inp.max_velocity = [rng.uniform(0.5, 3.0)]
    inp.max_acceleration = [rng.uniform(0.5, 3.0)]
    inp.max_jerk = [rng.uniform(1.0, 10.0)]
    return inp


def rand_brake(rng):
    """Brake regime: initial state deliberately beyond the limits."""
    inp = fresh_input(1)
    v_max = rng.uniform(0.5, 2.0)
    a_max = rng.uniform(0.5, 2.0)
    inp.current_position = [rng.uniform(-1, 1)]
    inp.current_velocity = [rng.uniform(-1.6, 1.6) * v_max]
    inp.current_acceleration = [rng.uniform(-1.6, 1.6) * a_max]
    inp.target_position = [rng.uniform(-2, 2)]
    inp.max_velocity = [v_max]
    inp.max_acceleration = [a_max]
    inp.max_jerk = [rng.uniform(1.0, 10.0)]
    return inp


def rand_multidof(rng):
    dofs = rng.choice([2, 3, 4])
    inp = fresh_input(dofs)
    inp.current_position = [rng.uniform(-1, 1) for _ in range(dofs)]
    inp.current_velocity = [rng.uniform(-0.4, 0.4) for _ in range(dofs)]
    inp.current_acceleration = [rng.uniform(-0.4, 0.4) for _ in range(dofs)]
    inp.target_position = [rng.uniform(-2, 2) for _ in range(dofs)]
    inp.max_velocity = [rng.uniform(0.3, 3.0) for _ in range(dofs)]
    inp.max_acceleration = [rng.uniform(0.3, 3.0) for _ in range(dofs)]
    inp.max_jerk = [rng.uniform(1.0, 10.0) for _ in range(dofs)]
    return inp


def rand_velocity_interface(rng):
    from ruckig import ControlInterface
    inp = fresh_input(1)
    inp.control_interface = ControlInterface.Velocity
    inp.current_velocity = [rng.uniform(-2, 2)]
    inp.current_acceleration = [rng.uniform(-1, 1)]
    inp.target_velocity = [rng.uniform(-2, 2)]
    inp.max_acceleration = [rng.uniform(0.5, 3.0)]
    inp.max_jerk = [rng.uniform(1.0, 10.0)]
    return inp


def rand_min_limits(rng):
    inp = fresh_input(1)
    v_max = rng.uniform(0.5, 2.0)
    a_max = rng.uniform(0.5, 2.0)
    inp.current_position = [rng.uniform(-1, 1)]
    inp.current_velocity = [rng.uniform(-0.3, 0.3)]
    inp.target_position = [rng.uniform(-2, 2)]
    inp.max_velocity = [v_max]
    inp.min_velocity = [-rng.uniform(0.2, 1.0) * v_max]
    inp.max_acceleration = [a_max]
    inp.min_acceleration = [-rng.uniform(0.2, 1.0) * a_max]
    inp.max_jerk = [rng.uniform(1.0, 10.0)]
    return inp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=Path("../src/test/resources/golden"))
    args = parser.parse_args()

    try_import_ruckig()
    args.out.mkdir(parents=True, exist_ok=True)

    cases = []
    cases += directed_cases()
    cases += random_cases("rand", 1, 50, rand_general)
    cases += random_cases("randA_general", 101, 150, rand_general)
    cases += random_cases("randB_brake", 102, 80, rand_brake)
    cases += random_cases("randC_multidof", 103, 60, rand_multidof)
    cases += random_cases("randD_velocity", 104, 40, rand_velocity_interface)
    cases += random_cases("randE_minlimits", 105, 40, rand_min_limits)

    payload = {
        "version": "0.17.3",
        "suite": "differential_v1",
        "cases": cases,
    }
    text = json.dumps(payload, indent=1)
    # Python emits bare Infinity/-Infinity (invalid JSON). Encode as 1e999,
    # which Double.parseDouble reads back as infinity on the Java side.
    text = text.replace("-Infinity", "-1e999").replace("Infinity", "1e999")
    out_path = args.out / "differential_v1.json"
    out_path.write_text(text, encoding="utf-8")
    print(f"Wrote {out_path} ({len(cases)} cases)")


if __name__ == "__main__":
    main()
