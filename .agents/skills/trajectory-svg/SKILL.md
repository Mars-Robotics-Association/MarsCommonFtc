---
name: trajectory-svg
description: "Generate SVG trajectory diagrams for documentation using ControlLab's BookSvgGenerator test class."
---

# Generating Trajectory SVG Diagrams

Use this when the user wants to create or update SVG plots of motion profiles for docs/book content.

## Key files

- **Generator**: `ControlLab/src/test/java/org/marsroboticsassociation/controllab/trajectory/BookSvgGenerator.java`
- **Output dir**: `book/` (set via `BOOK_DIR` in the generator)
- **Engine API**: `TrajectoryEngine` in the same package

## How to add a new SVG

1. Open `BookSvgGenerator.java` and add a new `@Test` method following the existing pattern:

```java
@Test
void generateMyProfile() throws IOException {
    TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
    engine.setPositionParams(/* vMax */ 10, /* aAccel */ 5, /* aDecel */ 5, /* jMax */ 20);
    engine.applyParamsAndGoTo(40.0);
    runToCompletion(engine);

    TrajectorySvgModel model = engine.buildExactSvgModel();
    writeSvg(BOOK_DIR.resolve("my-profile.svg"), model);
    engine.dispose();
}
```

2. Run the generator:
```
./gradlew :ControlLab:test --tests "*BookSvgGenerator"
```

## Available trajectory types

| Type | Constructor arg | Param setter | Notes |
|---|---|---|---|
| S-curve position | `TrajectoryType.SCURVE_POSITION` | `setPositionParams(vMax, aAccel, aDecel, jMax)` | Standard s-curve |
| Sinusoidal position | `TrajectoryType.SIN_CURVE_POSITION` | `setPositionParams(vMax, aAccel, aDecel, jMax)` | Sine-based acceleration |
| S-curve velocity | `TrajectoryType.SCURVE_VELOCITY` | `setVelocityParams(aMax, jInc, jDec)` | Velocity target, not position |
| Trapezoidal | `TrajectoryType.SCURVE_POSITION` | `setPositionParams(vMax, aAccel, aDecel, jMax)` | Use very high `jMax` (e.g. 100000) to eliminate jerk rounding |

## Tips

- **Keep target distances modest** relative to vMax so transitions (accel/decel/jerk phases) are visually prominent. Long cruise phases compress the interesting parts.
- The `writeSvg` and `runToCompletion` helpers are already in the class — reuse them.
- SVGs are built from exact analytical data via `buildExactSvgModel()`, not sampled ticks.
