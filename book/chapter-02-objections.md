# Chapter 2 Objections

Technical issues found by reviewing the chapter against the actual source code and hardware documentation.

---

## 1. IMotor has 7 methods, not 6

The chapter says "IMotor defines exactly six methods" and shows a code block with six signatures. The actual `IMotor.java` has seven methods -- `getPosition()` (returning encoder ticks) is missing from both the prose and the code listing.

**File:** `ControlLib/src/main/java/org/marsroboticsassociation/controllib/hardware/IMotor.java`

---

## 2. REV Hub communication is USB, not I2C

The chapter repeatedly describes the phone-to-hub link as "I2C or USB" (sections 2.4 and 2.9). The REV Expansion Hub and Control Hub communicate with the Android host over USB using a UART-over-USB protocol (FTDI). I2C is used on the hub side for sensors (IMU, color sensors, etc.), not for motor commands. Calling `setPower()` traffic "I2C" is incorrect -- it is USB bulk transfer. Students who read this and then look at bus analyzers or hub documentation will be confused.

Suggested fix: replace "I2C" references with "USB" when describing phone-to-hub motor command traffic. Reserve "I2C" for sensor discussions.

---

## 3. FtcMotors code listing is fabricated and has wrong formulas

The chapter's code block for `FtcMotors` diverges from the actual source in three ways:

**3a. `getGoBilda5000` body is wrong.** The chapter shows `return DCMotor.getNeoVortex(numMotors);` with a comment listing wrong specs (5200 RPM free speed, 3.7A stall current, 110 oz-in stall torque). The real code constructs a `new DCMotor(12.0, 0.1442, 9.2, 0.25, 607.4, numMotors)` with correct specs from the goBILDA datasheet (5800 RPM, 9.2A stall, 1.47 kgf-cm stall torque). Telling students the motor is modeled as a NEO Vortex (a brushless FRC motor) is misleading -- it is a Mabuchi RS-555 brushed motor with completely different characteristics.

**3b. `calcJ` formula is wrong.** The chapter shows:
```java
return kA * motor.KtNMPerAmp / (1.0 / motor.KvRadPerSecPerVolt);
```
This computes `kA * kT * Kv`, which is dimensionally wrong (units: V*s^2/rad * N*m/A * rad/s/V = N*m*s/A -- not kg*m^2). The actual code is:
```java
return kA * motor.KtNMPerAmp / motor.rOhms;
```
which correctly computes `kA * kT / R` (units: V*s^2/rad * N*m/A / ohm = kg*m^2).

**3c. `calcB` formula is missing the `1/R` factor.** The chapter shows:
```java
return motor.KtNMPerAmp * (kV - motor.KvRadPerSecPerVolt);
```
The actual code is:
```java
return (motor.KtNMPerAmp / motor.rOhms) * (kV - kE);
```
The missing division by `rOhms` makes the units wrong and the result off by a factor of R.

These are not simplifications for readability -- they are formula errors that would produce wrong physical parameters if a student tried to use them.

---

## 4. EncapsulatedDcMotorEx, QuantizedPowerMotor, and LinkedMotorGroup do not exist in ControlLib

The chapter presents these as core library classes, but none of them exist in the `ControlLib` source tree. The quantization logic actually lives in `MotorBase.setPower()` (in `ControlLib/src/main/java/org/marsroboticsassociation/controllib/control/MotorBase.java`). If these classes exist in a TeamCode project, the chapter should say so; as written, a student cloning the library will not find them.

---

## 5. SetOnChange API does not match the actual class

The chapter shows `SetOnChange` as having:
- A constructor taking an epsilon: `new SetOnChange<>(0.025)`
- A `boolean set(T newValue)` method that returns whether the value changed
- A separate `DoubleBackend` with a public `snap()` method

The actual `SetOnChange.java` has:
- Static factory methods: `SetOnChange.of()`, `SetOnChange.ofDouble()`
- A `void set(T value)` method that internally calls a setter callback (no boolean return)
- The `DoubleBackend` is a private inner class; `snap()` is private
- Construction requires passing a setter callback (e.g., `motor::setPower`)

The usage examples in section 2.9 would not compile against the real class.

---

## 6. SimMotorAdapter implements IMotor, not extends EncapsulatedDcMotorEx

Section 2.10 shows `SimMotorAdapter extends EncapsulatedDcMotorEx`. The actual test fixtures (`FlywheelTestFixture.java`, `ArmControllerTest.java`, `VerticalArmControllerTest.java`) all declare `SimMotorAdapter implements IMotor`. This matters because the whole point of the chapter is that depending on the narrow interface enables testability -- the test adapter should not need to extend a hardware-coupled class. The chapter's code contradicts its own thesis.

---

## 7. Motor voltage equation mixes up conventions

Section 2.8 presents:

$$V_{\text{applied}} = IR + k_E \omega$$

This is the steady-state motor voltage equation, which is fine. But the text then says "the feedforward voltage needed to achieve a target velocity is" and gives:

$$V_{ff} = k_S \text{sign}(\omega) + k_V \omega + k_A \alpha$$

The jump from the physics equation to the empirical feedforward model glosses over the fact that these are two different models. The first is a first-principles electrical equation; the second is an empirical fit with kS absorbing static friction, Coulomb friction, and other nonlinearities that IR + kE*omega does not capture. Presenting them sequentially as if the second follows from the first is misleading. Either explain the relationship (kV approximates kE when damping is small, kS captures friction the motor equation ignores, kA captures inertia) or just present the empirical model on its own.

---

## 8. "REV motor controller chip" language is imprecise

Section 2.9 says "the REV motor controller chip processes every command it receives." The REV Expansion Hub uses a TI processor running firmware that manages an H-bridge for each motor port. There is no separate "motor controller chip" per motor in the FRC/FTC sense (unlike, say, a Talon SRX). The hub firmware batches commands. This is a minor point, but "motor controller chip" suggests per-motor silicon that does not exist.
