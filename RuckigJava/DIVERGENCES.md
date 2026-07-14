# Divergences from upstream Ruckig v0.17.3

Per porting ground rule 4: never silently fix an upstream bug. Record intentional
differences here.

## None (expected)

The port aims for behavioral parity with Community `v0.17.3` (`cb99a04`). Golden-vector
and property tests are the source of truth.

## API / language surface (not algorithm)

These are unavoidable Java/API differences, not numerical divergences:

| Topic | Upstream | Java |
|---|---|---|
| DoF template | `Ruckig<DOFs>` compile-time + `DynamicDOFs` | Dynamic only (`double[]` sized at construction) |
| `check<CS, RL>` templates | compile-time `if constexpr` | enum parameters + runtime branches |
| `std::optional` | optional intervals / min duration | boolean flags + sentinel values |
| Value types | `Profile` copied by value | explicit `copyFrom` |
| Cloud / Pro | waypoints client | out of scope (not ported) |
