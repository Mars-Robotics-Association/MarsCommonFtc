# Porting log — Ruckig v0.17.3 → RuckigJava

Upstream: tag `v0.17.3`, SHA `cb99a04ce488f83701aaee6efd9c9f0d36a3d43b`.

| Java file | Upstream | Milestone | Notes |
|---|---|---|---|
| `Utils.java` | `utils.hpp` (+ cppMin/Max helpers) | M0 | Also hosts `DBL_EPSILON` |
| `Result.java` | `result.hpp` | M0 | int constants |
| `RuckigError.java` | `error.hpp` | M0 | |
| `ControlInterface.java` | `input_parameter.hpp` | M0 | |
| `Synchronization.java` | `input_parameter.hpp` | M0 | |
| `DurationDiscretization.java` | `input_parameter.hpp` | M0 | |
| `Roots.java` | `roots.hpp` | M1 | DoubleSet / PositiveDoubleSet / PairSet |
| `Bound.java` | `profile.hpp` (`Bound`) | M2 | |
| `BrakeProfile.java` | `brake.hpp` + `brake.cpp` | M3 | |
| `Profile.java` | `profile.hpp` | M2 | template checks → enums |
| `Block.java` | `block.hpp` | M4/M6 | |
| `VelocityThirdOrderStep1.java` | `velocity.hpp` + `velocity_third_step1.cpp` | M4 | |
| `VelocityThirdOrderStep2.java` | `velocity.hpp` + `velocity_third_step2.cpp` | M4 | |
| `VelocitySecondOrderStep1.java` | `velocity.hpp` + `velocity_second_step1.cpp` | M7 | |
| `VelocitySecondOrderStep2.java` | `velocity.hpp` + `velocity_second_step2.cpp` | M7 | |
| `PositionThirdOrderStep1.java` | `position.hpp` + `position_third_step1.cpp` | M5 | |
| `PositionThirdOrderStep2.java` | `position.hpp` + `position_third_step2.cpp` | M6 | |
| `PositionSecondOrderStep1.java` | `position.hpp` + `position_second_step1.cpp` | M7 | |
| `PositionSecondOrderStep2.java` | `position.hpp` + `position_second_step2.cpp` | M7 | |
| `PositionFirstOrderStep1.java` | `position.hpp` + `position_first_step1.cpp` | M7 | |
| `PositionFirstOrderStep2.java` | `position.hpp` + `position_first_step2.cpp` | M7 | |
| `TargetCalculator.java` | `calculator_target.hpp` | M6/M8 | |
| `Calculator.java` | `calculator.hpp` | M8 | Community: target only |
| `Trajectory.java` | `trajectory.hpp` | M4/M8 | single section |
| `InputParameter.java` | `input_parameter.hpp` | M8 | no Pro waypoint fields |
| `OutputParameter.java` | `output_parameter.hpp` | M8 | |
| `Ruckig.java` | `ruckig.hpp` | M8 | |

## Out of scope (not ported)

- `calculator_cloud.hpp`, `cloud_client.cpp`
- `python.cpp`
- Eigen / custom vector templates
