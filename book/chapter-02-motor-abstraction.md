# Chapter 2: Motor Abstraction & Hardware Interfaces

This chapter explores the `IMotor` interface — the thin boundary between control algorithms and hardware. You will see how wrapping the FTC SDK's `DcMotorEx` behind a seven-method interface enables testing, quantization, multi-motor groups, and battery voltage compensation without changing a single line of controller code.

## 2.1 The Problem with DcMotorEx

The FTC SDK provides `DcMotorEx`, an interface with over 60 methods covering power control, velocity control, current sensing, PIDF coefficients, encoder positions, and more. When controller code depends directly on `DcMotorEx`, several problems emerge:

**Testing is impossible.** `DcMotorEx` requires a physical REV Hub connected via USB. You cannot instantiate it in a JUnit test. Every controller that takes `DcMotorEx` as a parameter is locked to hardware.

**The interface is too wide.** A flywheel velocity controller needs `getVelocity()`, `setPower()`, and maybe `setVelocity()`. It does not need `getCurrent()`, `getMotorType()`, or `setPIDFCoefficients()`. When a class depends on 60 methods but uses 3, the dependency is unclear and the class is harder to reason about.

**Battery voltage is scattered.** Reading hub voltage requires reaching into `LynxModule` via reflection or SDK internals. If every controller does this independently, the logic is duplicated and inconsistent.

**Multi-motor mechanisms are awkward.** An arm driven by two mechanically coupled motors needs `setPower()` fanned out to both, but `getVelocity()` read from only one (or averaged). `DcMotorEx` has no concept of a "group."

`IMotor` solves all four problems with a narrow interface and a family of wrappers.

## 2.2 The IMotor Interface

`IMotor` defines exactly seven methods:

```java
public interface IMotor {
    double getVelocity();           // ticks per second
    double getPosition();           // encoder ticks
    void setPower(double power);    // -1.0 to 1.0
    void setVelocity(double tps);   // ticks per second (closed-loop)
    void setVelocityPIDFCoefficients(double p, double i, double d, double f);
    double getHubVoltage();         // volts, from parent LynxModule
    String getName();               // device name
}
```

Every controller in MarsCommonFtc — `FlywheelSimple`, `FlywheelStateSpace`, `VelocityMotorPF`, `VelocityMotorSdkPidf`, `ArmController` — depends on `IMotor`, not `DcMotorEx`. This means:

- Any implementation of `IMotor` can be swapped in: a real motor, a simulated motor, a logging decorator, a quantized wrapper.
- The interface documents exactly what a controller needs from hardware.
- Tests can provide a fake `IMotor` that returns synthetic velocity values.

## 2.3 EncapsulatedDcMotorEx: Bridging to the SDK

`EncapsulatedDcMotorEx` is the adapter that makes a real FTC motor implement `IMotor`. It implements both `DcMotorEx` and `IMotor`, delegating all 60+ `DcMotorEx` methods to an internal `DcMotorEx` instance:

```java
public class EncapsulatedDcMotorEx implements DcMotorEx, IMotor {
    private final DcMotorEx motor;
    private final LynxModule hub;
    private final String deviceName;

    public EncapsulatedDcMotorEx(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        hub = HubHelper.getHubForMotor(motor);
        deviceName = name;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();  // ticks per second
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getHubVoltage() {
        return hub.getInputVoltage(VOLTS);
    }

    // ... 60+ more delegated methods
}
```

The key method is `getHubVoltage()`. The REV Hub does not expose battery voltage through `DcMotorEx` — it is a property of the `LynxModule` itself. `EncapsulatedDcMotorEx` resolves the parent hub using `HubHelper.getHubForMotor()`, which uses reflection to find the `LynxModule` that owns the motor. This centralizes voltage reading in one place.

`EncapsulatedDcMotorEx` is the base of the wrapper chain. Every production motor starts here, then gets wrapped by additional layers that add behavior.

## 2.4 QuantizedPowerMotor: Reducing I2C Traffic

The FTC robot controller runs on an Android phone connected to a REV Expansion Hub via USB (UART-over-USB). Every `setPower()` call generates a command frame. (I2C is used on the hub side for sensors like the IMU and color sensors, not for motor commands.) If your control loop runs at 50 Hz and the power value changes by 0.0001 each cycle, you are flooding the bus with nearly identical commands.

`QuantizedPowerMotor` sits between the controller and the hardware, rounding power values to a fixed step and only sending commands when the quantized value actually changes:

```java
public class QuantizedPowerMotor extends EncapsulatedDcMotorEx {
    private final double quantStep;
    private double lastPower = Double.NaN;

    @Override
    public void setPower(double power) {
        power = MathUtil.clamp(power, -1.0, 1.0);
        double quantPower = Math.round(power / quantStep) * quantStep;
        if (Double.isNaN(lastPower) || quantPower != lastPower) {
            super.setPower(quantPower);
            lastPower = quantPower;
        }
    }
}
```

With `quantStep = 0.01`, power values are rounded to the nearest 1%. A command that changes from 0.354 to 0.356 produces the same quantized value (0.35) and no I2C transaction occurs. A change from 0.354 to 0.365 rounds to 0.36 and triggers a write.

Usage:

```java
EncapsulatedDcMotorEx rawMotor = new EncapsulatedDcMotorEx(hardwareMap, "flywheel");
QuantizedPowerMotor motor = new QuantizedPowerMotor(rawMotor, 0.01);
```

`QuantizedPowerMotor` also has a constructor that takes a `HardwareMap` and motor name directly, building the `EncapsulatedDcMotorEx` internally.

## 2.5 LinkedMotorGroup: Multi-Motor Mechanisms

Many mechanisms use two or more mechanically coupled motors: a dual-motor flywheel, a four-motor drivetrain, a dual-arm with left and right sides. `LinkedMotorGroup` treats them as a single `IMotor`:

```java
public class LinkedMotorGroup extends EncapsulatedDcMotorEx {
    private final DcMotorEx[] motors;
    private final DcMotorSimple.Direction[] directions;

    public LinkedMotorGroup(HardwareMap hardwareMap, MotorConfig... configs) {
        super(hardwareMap, configs[0].name());  // primary motor
        motors = new DcMotorEx[configs.length];
        directions = new DcMotorSimple.Direction[configs.length];
        for (int i = 0; i < configs.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, configs[i].name());
            directions[i] = configs[i].direction();
        }
    }
}
```

`MotorConfig` is a simple record pairing a name with a direction:

```java
record MotorConfig(String name, DcMotorSimple.Direction direction) {}
```

Write operations fan out to all motors:

```java
@Override
public void setPower(double power) {
    for (int i = 0; i < motors.length; i++) {
        motors[i].setPower(power * (directions[i] == REVERSE ? -1 : 1));
    }
}

@Override
public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
    for (DcMotorEx m : motors) {
        m.setZeroPowerBehavior(behavior);
    }
}
```

Read operations come from the primary motor (the first in the list):

```java
@Override
public double getVelocity() {
    return motors[0].getVelocity();
}

@Override
public double getCurrent() {
    return motors[0].getCurrent();
}
```

`LinkedMotorGroup` explicitly blocks closed-loop modes. You cannot call `setVelocity()` or `setTargetPosition()` on a group, because the SDK's closed-loop control runs independently on each motor's controller chip, and there is no way to synchronize them:

```java
@Override
public void setVelocity(double tps) {
    throw new UnsupportedOperationException(
        "LinkedMotorGroup does not support closed-loop velocity control");
}
```

This is a deliberate design choice: if you need closed-loop velocity control on multiple motors, use individual `VelocityMotorPF` or `VelocityMotorSdkPidf` controllers for each motor rather than grouping them.

Usage:

```java
LinkedMotorGroup group = new LinkedMotorGroup(hardwareMap,
    new MotorConfig("flywheel",  DcMotorSimple.Direction.REVERSE),
    new MotorConfig("flywheel2", DcMotorSimple.Direction.REVERSE));
group.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
group.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
```

## 2.6 The Complete Wrapper Chain

A typical production motor setup chains multiple wrappers:

```java
// Layer 1: Raw SDK adapter
EncapsulatedDcMotorEx raw = new EncapsulatedDcMotorEx(hardwareMap, "flywheel");

// Layer 2: Quantized power output
QuantizedPowerMotor motor = new QuantizedPowerMotor(raw, 0.01);

// Layer 3: Controller
FlywheelSimple flywheel = new FlywheelSimple(telemetry::addData, motor);
```

Or with a motor group:

```java
LinkedMotorGroup group = new LinkedMotorGroup(hardwareMap,
    new MotorConfig("flywheel",  REVERSE),
    new MotorConfig("flywheel2", REVERSE));
group.setMode(RUN_WITHOUT_ENCODER);
group.setZeroPowerBehavior(FLOAT);

QuantizedPowerMotor motor = new QuantizedPowerMotor(group, 0.01);
FlywheelSimple flywheel = new FlywheelSimple(telemetry::addData, motor);
```

Each layer adds one concern:

| Layer | Concern |
|---|---|
| `EncapsulatedDcMotorEx` | SDK adaptation, hub voltage reading |
| `LinkedMotorGroup` | Multi-motor fan-out, direction handling |
| `QuantizedPowerMotor` | Power quantization, change detection |
| `FlywheelSimple` (or any controller) | Control algorithm |

The controller at the top does not know or care how many motors are underneath, whether power is quantized, or how voltage is read. It calls `motor.getVelocity()` and `motor.setPower()` — that is the entire contract.

## 2.7 FtcMotors: Motor Characterization Constants

`FtcMotors` is a utility class that provides WPILib `DCMotor` models and mechanical calculations:

```java
public class FtcMotors {
    public static DCMotor getGoBilda5000(int numMotors) {
        // goBILDA 5000-series (Mabuchi RS-555)
        // 12V: 5800 RPM free speed, 9.2A stall, 1.47 kgf-cm stall torque
        return new DCMotor(12.0, 0.1442, 9.2, 0.25, 607.4, numMotors);
    }

    public static double calcJ(DCMotor motor, double kA) {
        // J = kA * kT / R
        // System inertia from characterized acceleration constant
        return kA * motor.KtNMPerAmp / motor.rOhms;
    }

    public static double calcB(DCMotor motor, double kV, double kE) {
        // b = (kT/R) * (kV - kE)
        // Viscous damping from characterized velocity constant
        return (motor.KtNMPerAmp / motor.rOhms) * (kV - kE);
    }
}
```

These methods bridge the gap between empirical characterization (finding kS, kV, kA through testing) and physical parameters (inertia J, damping b) needed for state-space models. Chapter 3 covers feedforward characterization in detail; Chapter 12 covers how these physical parameters feed into state-space design.

## 2.8 Battery Voltage Compensation

Battery voltage sags under load and declines as the match progresses. A power command of 0.5 at 13.0 V produces less torque than 0.5 at 11.5 V. Every controller in MarsCommonFtc compensates for this by reading live hub voltage and scaling its output.

### The Voltage Balance Equation

A DC motor obeys:

$$V_{\text{applied}} = IR + k_E \omega$$

Where $V_{\text{applied}}$ is the voltage applied to the motor terminals, $I$ is current, $R$ is winding resistance, $k_E$ is the back-EMF constant, and $\omega$ is angular velocity. In practice, the first-principles equation above doesn't capture everything — static friction, Coulomb friction, and other nonlinearities also matter. The empirical feedforward model used in Chapter 3 accounts for these:

$$V_{ff} = k_S \mathop{\text{sign}}(\omega) + k_V \omega + k_A \alpha$$

Where $k_S$ absorbs static friction and other nonlinearities the motor equation ignores, $k_V$ approximates $k_E$ (plus any velocity-dependent losses), and $k_A$ captures the inertial term. To convert this voltage to a power command:

$$\text{power} = \frac{V_{ff}}{V_{\text{battery}}}$$

This simple division normalizes the command so that the same velocity target produces the correct terminal voltage regardless of battery state.

### Live Voltage Reading

Each controller reads voltage at different times:

**FlywheelSimple** captures voltage once at construction:

```java
public FlywheelSimple(TelemetryAddData telemetry, IMotor motor) {
    this.motor = motor;
    this.hubVoltage = motor.getHubVoltage();  // captured once
}
```

This is the simplest approach but does not track voltage sag during a match.

**VelocityMotorPF** reads voltage when the target changes or acceleration is zero:

```java
private void onTargetChange() {
    voltage = getVoltage();  // live reading
    vmax = (1 - headroomAllowance) * (voltage - kS) / kV;
}

@Override
public void update() {
    // ... trajectory update
    if (acceleration == 0) {
        voltage = getVoltage();  // refresh at steady state
    }
    double ff = (kS * sign(velocity) + kV * velocity + kA * acceleration) / voltage;
}
```

**VelocityMotorSdkPidf** uses `SetOnChange` to update the F coefficient only when voltage changes beyond a threshold:

```java
private final SetOnChange<Double> voltageFactor = new SetOnChange<>(0.025);

@Override
public void update() {
    double vf = Kv * nominalVoltage / getVoltage();
    voltageFactor.set(vf);  // only writes PIDF coefficients if vf changed beyond threshold
}
```

This avoids redundant USB writes to the hub while still tracking voltage changes.

**FlywheelStateSpace** reads voltage every cycle and clamps the LQR output:

```java
@Override
public void update() {
    double hubVoltage = motor.getHubVoltage();
    double u = -K.times(xHat).get(0, 0);  // LQR control voltage
    u = MathUtil.clamp(u, -hubVoltage, hubVoltage);
    motor.setPower(u / hubVoltage);
}
```

### Detecting Insufficient Voltage

`FlywheelSimple` includes a diagnostic that warns when the feedforward voltage approaches the available battery voltage:

```java
public boolean isPowerTooLowForTargetVelocity() {
    double ffFraction = Math.abs(ffVoltage) / hubVoltage;
    return ffFraction > 0.85;
}
```

When this returns `true`, the controller is demanding more than 85% of available voltage just for feedforward — there is no headroom left for feedback correction. The target velocity is unachievable at the current battery state.

## 2.9 SetOnChange: Avoiding Redundant Writes

`SetOnChange` is a utility that tracks a value and calls a setter callback only when the value has changed beyond a threshold. It is used extensively to avoid redundant hardware writes:

```java
// Created via static factory methods with a setter callback
SetOnChange<Double> velocitySetpoint = SetOnChange.ofDouble(1.0, motor::setVelocity);
SetOnChange<Double> voltageFactor = SetOnChange.ofDouble(0.025, newFactor ->
    motor.setVelocityPIDFCoefficients(kP, kI, kD, newFactor));
```

Usage in `VelocityMotorSdkPidf`:

```java
velocitySetpoint.set(targetTps);   // only calls motor::setVelocity if changed by > 1.0
voltageFactor.set(newFactor);      // only calls setPIDFCoefficients if changed by > 0.025
```

This pattern matters because the REV Hub firmware processes every command it receives. Sending the same PIDF coefficient 50 times per second wastes USB bandwidth and can introduce jitter.

## 2.10 Testing with SimMotorAdapter

The `IMotor` interface enables testing controllers without hardware. `SimMotorAdapter` bridges between `IMotor` and `FlywheelMotorSim`:

```java
// In TeamCode test
public class SimMotorAdapter implements IMotor {
    private final FlywheelMotorSim sim;

    public SimMotorAdapter(FlywheelMotorSim sim) {
        this.sim = sim;
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityTPS();
    }

    @Override
    public void setPower(double power) {
        sim.setInputVoltage(power * 12.0);  // simulate 12V battery
    }

    @Override
    public double getHubVoltage() {
        return 12.0;
    }
}
```

The test creates a simulated motor, passes it to the controller, and steps the simulation:

```java
@Test
void testFlywheelReachesTarget() {
    FlywheelMotorSim sim = new FlywheelMotorSim(kV, kA, 12.0);
    SimMotorAdapter motor = new SimMotorAdapter(sim);
    FlywheelSimple controller = new FlywheelSimple(NoOpTelemetry.INSTANCE, motor);

    controller.setTargetVelocity(3000);

    for (int i = 0; i < 200; i++) {
        controller.update();
        sim.update(0.02);  // 50 Hz loop
    }

    assertEquals(3000, sim.getVelocityTPS(), 50);
}
```

This same pattern works for every controller that accepts `IMotor`. The test runs in milliseconds on your development machine, no robot required.

## 2.11 Summary

The motor abstraction layer in MarsCommonFtc follows a simple principle: **narrow interface, composable wrappers**. `IMotor` defines the minimum contract between controller and hardware. `EncapsulatedDcMotorEx` adapts the SDK. `QuantizedPowerMotor` reduces bus traffic. `LinkedMotorGroup` handles multi-motor mechanisms. Each wrapper adds one concern without affecting the others.

Controllers depend only on `IMotor`, which means:

- They work with any motor implementation — real, simulated, or decorated.
- Battery voltage compensation is consistent across all controllers.
- Tests run on the development machine with simulated motors.
- The control algorithm code is completely decoupled from hardware details.

With the hardware interface established, the next chapter examines the first layer of control: feedforward models that predict the voltage needed to achieve a desired motion.
