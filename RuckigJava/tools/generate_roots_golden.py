#!/usr/bin/env python3
"""Generate roots golden vectors by evaluating the same formulas in pure Python
is NOT valid for differential testing — this script embeds the C++ algorithm
output format and expects an external oracle.

Preferred: compile tools/roots_oracle.cpp against vendored ruckig headers and
pipe its JSON. This Python helper only writes the *schema* sample used by M1
when the C++ oracle is unavailable; residual tests in RootsTest cover structure.
"""

import json
import random
from pathlib import Path


def main():
    rng = random.Random(123)
    cases = []
    for i in range(100):
        a = rng.uniform(-5, 5)
        b = rng.uniform(-5, 5)
        c = rng.uniform(-5, 5)
        d = rng.uniform(-5, 5)
        cases.append({
            "id": f"cubic_{i:03d}",
            "type": "cubic",
            "coeffs": [a, b, c, d],
            # roots filled by C++ oracle; empty means residual-only
            "roots": []
        })
    out = Path(__file__).resolve().parent.parent / "src/test/resources/golden/roots_schema.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"version": "0.17.3", "suite": "roots", "cases": cases}, indent=2), encoding="utf-8")
    print("Wrote", out)


if __name__ == "__main__":
    main()
