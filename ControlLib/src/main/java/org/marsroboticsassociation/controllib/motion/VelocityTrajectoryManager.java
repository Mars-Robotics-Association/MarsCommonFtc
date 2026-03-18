package org.marsroboticsassociation.controllib.motion;

import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import java.util.function.LongSupplier;

public class VelocityTrajectoryManager {
    private double aMax;
    private double jInc;
    private double jDec;

    private final LongSupplier clock;
    private final TelemetryAddData telemetry;
    private final SetOnChange<Double> targetVelocity;
    private VelocityTrajectory currentTrajectory;
    private long startTime;
    private double lastV;
    private double lastA;
    private double pendingTarget;

    public VelocityTrajectoryManager(double aMax, double jMax, double vChangeTolerance, TelemetryAddData telemetry) {
        this(aMax, jMax, vChangeTolerance, telemetry, System::nanoTime);
    }

    /** For unit tests — allows injecting a simulated clock. */
    public VelocityTrajectoryManager(double aMax, double jMax, double vChangeTolerance, TelemetryAddData telemetry, LongSupplier clock) {
        this.clock = clock;
        this.telemetry = telemetry;
        pendingTarget = Double.NaN;
        updateConfig(aMax, jMax, jMax);
        lastV = 0.0;
        lastA = 0.0;
        startTime = clock.getAsLong();
        changeTrajectory(0);
        targetVelocity = SetOnChange.ofDouble(0.0, vChangeTolerance, (v) -> {
            pendingTarget = v;
            update();
        });
    }

    public void updateConfig(double aMax, double jInc, double jDec) {
        this.aMax = Math.abs(aMax);
        this.jInc = Math.abs(jInc);
        this.jDec = Math.abs(jDec);
    }

    /**
     * Create a new SCurveVelocity trajectory from the current state to the given target velocity.
     * This method safely handles opposing accelerations and tiny Δv.
     */
    private void changeTrajectory(double vTarget) {
        double dv = vTarget - lastV;

        // If dv is effectively zero, create a trivial trajectory
        if (Math.abs(dv) < 1e-6) {
            currentTrajectory = new SCurveVelocity(lastV, lastV, 0.0, aMax, jInc, jDec);
        } else {
            // Compute effective starting acceleration in the direction of the target
            double dir = Math.signum(dv);
            double effectiveA0 = lastA;

            // Clamp initial acceleration to not overshoot the new target immediately
            if (dir * effectiveA0 > aMax) {
                effectiveA0 = dir * aMax;
            }

            currentTrajectory = new SCurveVelocity(lastV, vTarget, effectiveA0, aMax, jInc, jDec);
        }

        startTime = clock.getAsLong();
        pendingTarget = Double.NaN;
    }

    /**
     * Update current velocity/acceleration (call once per loop)
     */
    public void update() {
        double seconds = (clock.getAsLong() - startTime) / 1e9;

        lastV = currentTrajectory.getVelocity(seconds);
        lastA = currentTrajectory.getAcceleration(seconds);

        telemetry.addData("trajectory velocity", "%.3f", lastV);
        telemetry.addData("trajectory acceleration", "%.1f", lastA);

        // Always update trajectory if target changed
        if (!Double.isNaN(pendingTarget)) {
            this.changeTrajectory(pendingTarget);
        }
    }

    /**
     * Immediately discard the current trajectory and restart from a measured velocity.
     * Acceleration is assumed zero.
     */
    public void resetFromMeasurement(double measuredV) {
        resetFromMeasurement(measuredV, 0.0);
    }

    /**
     * Immediately discard the current trajectory and restart from measured velocity & acceleration.
     */
    public void resetFromMeasurement(double measuredV, double measuredA) {
        // Force-update internal state
        this.lastV = measuredV;
        this.lastA = measuredA;

        // Discard any pending trajectory-update request
        this.pendingTarget = Double.NaN;

        // Rebuild trajectory from measured conditions to current target
        double currentTarget = targetVelocity.get();

        changeTrajectory(currentTarget);
    }


    /**
     * Command a new target velocity at any time
     */
    public void setTarget(double vTarget) {
        targetVelocity.set(vTarget);
    }

    public double getTarget() {
        return targetVelocity.get();
    }

    public double getVelocity() {
        return lastV;
    }

    public double getAcceleration() {
        return lastA;
    }
}
