# FTC Programming: A Practical Guide with MarsCommonFtc

## Table of Contents

---

## Part I: Foundations

### [Chapter 1: Getting Started with FTC Java Development](chapter-01-getting-started.md)
- Project structure and module organization
- Git submodules for shared code
- Gradle setup and build workflow
- The MarsCommonFtc architecture

### [Chapter 2: Motor Abstraction & Hardware Interfaces](chapter-02-motor-abstraction.md)
- The `IMotor` hardware-agnostic interface
- `FtcMotors` constants and motor characterization
- Quantized power updates to reduce command noise
- Battery voltage compensation

### [Chapter 3: Feedforward Control Theory](chapter-03-feedforward-control.md)
- The kS/kV/kA model
- Back-EMF physics and the voltage balance equation
- `SimpleMotorFeedforward` for velocity systems
- `ArmFeedforward` with gravity compensation
- `ElevatorFeedforward` for constant-gravity mechanisms
- Characterizing your mechanism to find feedforward gains

### [Chapter 4: PID Control](chapter-04-pid-control.md)
- How `PIDController` works
- Tuning kP, kI, and kD
- Continuous input for rotational mechanisms
- Setpoint ramping
- When PID is enough — and when it isn't

---

## Part II: Motion Profiling

### [Chapter 5: Trapezoidal & Exponential Profiles](chapter-05-trapezoidal-exponential-profiles.md)
- `TrapezoidProfile` — constant acceleration motion
- `ExponentialProfile` — first-order system response
- When to use each profile type

### [Chapter 6: S-Curve Jerk-Limited Trajectories](chapter-06-s-curve-trajectories.md)
- Why jerk matters for mechanism longevity
- `SCurvePosition` — 7-segment position profiles
- Asymmetric acceleration and deceleration limits
- `SCurveVelocity` — velocity-to-velocity profiles
- Handling non-zero initial conditions

### Chapter 7: Sinusoidal Trajectories
- `SinCurvePosition` — raised-cosine acceleration curves
- Eliminating jerk discontinuities at phase boundaries
- Comparing S-curve vs. sinusoidal profiles

### [Chapter 8: Online Trajectory Generation with Ruckig](chapter-08-ruckig.md)
- What is online trajectory generation?
- The Ruckig library overview
- Multi-DOF real-time planning
- JNI integration with `RuckigController`

### [Chapter 9: Trajectory Management & Replanning](chapter-09-trajectory-management.md)
- `PositionTrajectoryManager` and `VelocityTrajectoryManager`
- Mid-motion replanning with continuity guarantees
- Handling disturbances and tracking errors
- Telemetry and debugging

### [Chapter 10: Back-EMF-Aware Profile Tuning](chapter-10-back-emf-aware-tuning.md)
- Why back-EMF limits achievable acceleration
- `findMaxJDec()` and `findMaxAMax()` binary search methods
- Voltage-constrained acceleration limits
- Auto-tuning profiles in ControlLab

---

## Part III: State-Space Control

### [Chapter 11: Linear Algebra for Robotics](chapter-11-linear-algebra.md)
- `Matrix` and `Vector` operations
- Type-level dimensions with `Nat` and `Num`
- The Discrete Algebraic Riccati Equation (`DARE`) solver
- Building and composing linear systems

### [Chapter 12: State-Space Fundamentals](chapter-12-state-space-fundamentals.md)
- Plant models and state-space representation
- `LinearSystemId` — identifying systems from physical parameters
- Continuous-to-discrete conversion
- The `KalmanFilter` observer
- The `LinearQuadraticRegulator` (LQR)

### [Chapter 13: Flywheel State-Space Controller](chapter-13-flywheel-state-space.md)
- `FlywheelStateSpace` — LQR + Kalman observer
- Handling FTC's non-deterministic loop timing
- Comparing state-space to PID + feedforward

### Chapter 14: Advanced State Estimation
- `ExtendedKalmanFilter` for nonlinear systems
- `UnscentedKalmanFilter` with Merwe sigma points
- `FlywheelBallTracker` — augmented state with ball-drag disturbance detection
- Hysteresis-based event detection

---

## Part IV: Mechanism Control

### Chapter 15: Arm Control: Feedforward + PD + Kalman
- `ArmController` architecture
- S-curve trajectory generation for arm moves
- Gravity compensation via feedforward
- Latency compensation with the Kalman filter
- Hard stop coasting logic
- Per-move gravity-aware trajectory limits
- Trajectory replanning on tracking error

### Chapter 16: Arm Control: Feedback Linearization + LQR
- `VerticalArmController` — the three-layer design
- Layer 1: Canceling gravity and friction
- Layer 2: LQR + linear feedforward on the linearized plant
- Layer 3: S-curve trajectory management
- The gravity-corrected Kalman input
- LQR auto-tuning vs. manual PD gains
- The saturation-estimation coupling constraint

### Chapter 17: Velocity Motor Controllers
- `VelocityMotorPF` — proportional + feedforward control
  - Acceleration-dependent kP scaling
  - Jerk-limited motion profiling
  - Live battery voltage compensation
- `VelocityMotorSdkPidf` — wrapping the SDK's built-in PIDF
  - `RUN_USING_ENCODER` mode
  - `SetOnChange` to avoid redundant I2C writes

---

## Part V: Localization & Odometry

### Chapter 18: Robot Kinematics & Odometry
- `ChassisSpeeds` — robot-relative velocities
- `DifferentialDriveKinematics` — tank/skid-steer
- `MecanumDriveKinematics` — omni-directional drive
- `Odometry` and `Odometry3d` — pose tracking from wheel positions
- Twist2d and differential motion

### Chapter 19: Sensor Fusion & Pose Estimation
- `FieldPoseEstimator` — latency-compensated Kalman filtering
- `PinpointOdometry` — working with odometry computers
- `PinpointPoseEstimator` — fusing odometry with vision
- Anisotropic (per-axis) vision standard deviations
- Odometry history buffers for latency compensation

---

## Part VI: Utilities & Tooling

### Chapter 20: Signal Filtering
- `IIR1LowPassVarDt` — first-order low-pass with exact discrete-time solution
- `BiquadLowPassVarDt` — second-order low-pass with matrix exponential discretization
- `LinearFilter` — FIR filters and moving averages
- `MedianFilter` — outlier rejection
- `SlewRateLimiter` — preventing sudden signal jumps
- Variable-dt filtering for non-deterministic loops

### Chapter 21: Lookup Tables & Interpolation
- `LUT` — nearest-neighbor lookup with `TreeMap`
- `LinInterpTable` — linear interpolation between entries
- `InterpLUT` — monotone cubic Hermite spline interpolation (Fritsch-Carlson)
- `InterpolatingTreeMap` — automatic interpolation between entries
- `TimeInterpolatableBuffer` — time-indexed buffered interpolation

### Chapter 22: Projectile Motion
- Fixed-angle launch calculations
- `getLaunchSpeed()` — solving for required velocity at a given distance
- `getMaxTrajectoryHeight()` — peak height of the projectile arc
- Distance-based flywheel RPM correction
- Motion-compensated shooting while the robot is moving

### Chapter 23: Physics Simulation
- `EncoderSim` — faithful REV Hub encoder ring-buffer model
  - 10 ms sampling, 50 ms velocity window, 20 TPS quantization
- `FlywheelMotorSim` — first-order linear velocity dynamics with RK4 integration
- `ArmMotorSim` — nonlinear arm dynamics with gravity and hard stops
- Disturbance voltage injection for testing
- Building test fixtures for controller validation

### Chapter 24: Desktop Visualization & Tuning with ControlLab
- The ControlLab Swing application
- Filter tab: CSV signal analysis, filter application, lag estimation via cross-correlation
- Trajectory tab: interactive visualization, limit sliders, back-EMF violation detection, SVG export
- Flywheel tab: real-time simulation, controller comparison, plant randomization game
- Using simulation to validate controllers before on-robot testing
