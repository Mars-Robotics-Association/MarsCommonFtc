package org.marsroboticsassociation.controllib.hardware

import edu.wpi.first.math.system.plant.DCMotor
import kotlin.math.PI

/**
 * WPILib [DCMotor] constants for FTC-legal DC motors.
 *
 * <p>Use these with `LinearSystemId.createDCMotorSystem` or WPILib simulation classes. Constants
 * represent the bare motor before any gearbox; apply gear ratios separately.
 */
object FtcMotors {

    /**
     * Returns a [DCMotor] model for the Mabuchi RS-555 brushed DC motor — the base motor inside all
     * goBILDA Yellow Jacket Planetary Gear Motors (5203 series and others) and the bare goBILDA
     * 5000-series motor. Several other FTC vendors use the same motor core.
     *
     * <p>This represents the bare motor before any planetary gearbox. When modelling a geared
     * Yellow Jacket, account for the gear ratio in the system model (e.g. in the moment of inertia
     * or via `LinearSystemId`).
     *
     * <p>Specs from the goBILDA 5000-series datasheet (tested values at 12V). Stall current and
     * free current are confirmed by the 5203-series Yellow Jacket datasheet; the 5000-series stall
     * torque is preferred over the back-calculated Yellow Jacket value as it is a direct
     * measurement without gearbox efficiency losses.
     * - Free speed: 5800 RPM (tested); 6000 RPM theoretical
     * - Stall torque: 1.47 kgf·cm (0.1442 N·m)
     * - Stall current: 9.2 A
     * - Free current: 0.25 A
     *
     * @param numMotors number of motors in the gearbox
     */
    fun getGoBilda5000(numMotors: Int): DCMotor {
        return DCMotor(
            12.0,
            1.47 * 0.0980665, // 1.47 kgf·cm → N·m (1 kgf·cm = 9.80665N × 0.01m)
            9.2,
            0.25,
            5800.0 * 2.0 * PI / 60.0, // RPM → rad/s
            numMotors,
        )
    }

    /**
     * Computes the total system moment of inertia (kg·m²) from a characterized `kA` (V·s²/rad) and
     * motor specs.
     *
     * <p>Derivation: from the velocity-system model `dω/dt = (kT/RJ)·V − …`, `kA = R·J/kT`, so `J =
     * kA·kT/R = kA·τ_stall/V_nominal`.
     *
     * <p>The returned J is the *total* reflected inertia of the characterized mechanism (motor
     * rotor
     * + load), not the rotor alone.
     *
     * @param motor the motor model (e.g. from [getGoBilda5000])
     * @param kA characterized kA in V·s²/rad (WPILib SI units)
     * @return total system moment of inertia in kg·m²
     */
    fun calcJ(motor: DCMotor, kA: Double): Double {
        return kA * motor.KtNMPerAmp / motor.rOhms
    }

    /**
     * Computes the viscous damping coefficient (N·m·s/rad) from characterized `kV` and `kA`
     * (V·s/rad and V·s²/rad) and motor specs.
     *
     * <p>Derivation: `kV/kA = (kT·kE/R + b)/J`. Substituting `J = kA·kT/R` and solving: `b =
     * (kT/R)·(kV − kE)`, where `kE = 1/KvRadPerSecPerVolt`.
     *
     * <p>If the result is near zero, viscous damping is negligible and `kV` is dominated by
     * back-EMF — typical for flywheels with good bearings. A negative result indicates measurement
     * noise or model mismatch.
     *
     * @param motor the motor model (e.g. from [getGoBilda5000])
     * @param kV characterized kV in V·s/rad (WPILib SI units)
     * @return viscous damping coefficient in N·m·s/rad
     */
    fun calcB(motor: DCMotor, kV: Double): Double {
        val kE = 1.0 / motor.KvRadPerSecPerVolt
        return (motor.KtNMPerAmp / motor.rOhms) * (kV - kE)
    }
}
